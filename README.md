# MayDarkness2026

This project is a project testing how far we can send messages from an Arduino Nano using it's GPIO pins connected to a WiFi antenna from a router and a coper plate, not impedance matching has been performed.
The idea:

Arduino Nano has no WiFi or Bluetooth but is a perfectly good microcontroller, could we have a nano in end of a table send messages to an ESP32 or a Raspberry Pi Pico W on the other end of the room, which can then forward it over the internet.

The setup:
- You have access to a arduino nano over USB Serial.
- Pin D10 of the nano is connected to a WiFi antenna from a router
- Pin D11 is connected to a 4cmx3cm copper plate

- You also have access to a raspberry pi pico W over USB
- It's GPIO28 is connected to a similar copper plate
- It's GPIO27 is connected to a WiFi antenna from a router

- You also have access to an ESP32 over USB
- GPIO34 is connected to a long jumper wire ~20cm which can act as an antenna
- There is the esp-sdr/ project which lets you get the raw samples from the WiFi radio on the ESP

What has been done:
Under NanoWireless/chirp-sender runs on the Arduino Nano and sends LoRa like chirp signals using the GPIO
chirp-sdr and chirp-sdr-esp run a simple SDR that plots the readings from the GPIO pin
chirp-receiver tries to decode the chirp
The setup works for up to about 15cm distance between the Pico W and Nano.


The setup:
Right now the Nano is placed 25cm away from the ESP32 and Pico W, come up with a method to transmit data from the Nano to either or both of the other microcontollers (ESP32/Pico W). The Pico W has a better ADC so the antenna, and copper plate is connected to it. While the ESP has access to the raw WiFi radio samples (this goes down to 1 Ghz). Try things out until you manage to sucesfully transmit some bytes between them. Utilize all the different physical properties of RF waves. For example, if we send a square wave using the Nano, it's harmonics might land in the 1Ghz range and the esp might be able to find it, or if we can get the radio to find a lower harmonic of the 2.4Ghz antennas. Or using "capacitance" using the copper plates.

Port the esp-sdr to platformio if it makes your life easier. Install whatever toolchain you want. You have access to platformIO to reprogram and monitor all the microcontrollers.

Goodluck!

---

# Update: setup audit and new experiment ideas

Constraints still apply: **no new components**. Jumper wires between pins that already exist are fine; the point is to see how far this exact setup can be pushed.

> How this section was produced: a static read of the uploaded `NanoWireless` archive (`chirp-sender`, `chirp-receiver`, `chirp-sdr`, `chirp-sdr-esp`). No hardware was run. Numbers tagged **(est.)** are back-of-envelope and need to be measured. `esp-sdr/` is mentioned above but was not in the archive, so nothing here reviews it.

---

## Part A. Audit: oversights and gaps in the current setup

Read this before running anything new. Several of these could explain why the link dies past ~15 cm.

### A1. Firmware vs. wiring mismatches (cheap to check, check first)

- **Pico pin.** The README wires the copper plate to GPIO28 and the antenna to GPIO27. Every Pico firmware (`chirp-receiver`, `chirp-sdr`) reads **GP26 / ADC0** (`adc_gpio_init(26)`, `adc_select_input(0)`). GPIO27 is ADC1 and GPIO28 is ADC2. Unless a third wire sits on GP26, the receiver may be sampling a floating pin.
- **ESP32 pin.** The README says the jumper is on GPIO34 (ADC1_CH6). `chirp-sdr-esp` samples `ADC1_CHANNEL_0`, which is **GPIO36**.
- **Nano antenna unused.** The sender only drives D11 (OC2A). D10 (the router antenna) is never driven, so half of the transmit hardware is idle.
- **Protocol constants disagree between sender and every receiver,** although the comments say they "must match":

| constant | sender | chirp-receiver (Pico) | chirp-sdr (Pico) | chirp-sdr-esp |
|---|---|---|---|---|
| PREAMBLE_MS | 2000 | 500 | 2000 | 2000 |
| DATA_MS | 1000 | 750 | 1000 | 1000 |
| symbol repeats | 3 | 3 | 1 | 1 |
| PREAMBLE_GAP_MS | 1000 | 100 | 1500 | 1500 |
| MIN_CORRELATION | n/a | 0.01 | 0.2 | 0.2 |

These may be mid-edit snapshots, but no receiver build in the archive matches the sender as it stands. Re-verify the whole chain end to end before trusting any "it works at 15 cm" result.

### A2. The sender does not sweep what its comments say

- Timer2 CTC toggle: `f = 16 MHz / (2 * (OCR2A + 1))`.
- `OCR_LOW = 64` gives about **123 kHz**; `OCR_HIGH = 48` gives about **163 kHz**. The header comments still describe an older revision (157 kHz up to 8 MHz, "strong-coupling high-frequency band").
- OCR2A only takes **17 distinct values** between 64 and 48, so the "256-step sweep" is a 17-step staircase with 2-3 kHz steps, and the whole sweep spans only about 1.3:1 in frequency.
- Net effect: the MHz range, where wire/plate resonances and the ESP's harmonic reception could matter, has never been exercised by the sender.

### A3. The amplitude-trend decoder rests on a shaky premise

- The premise is "higher frequency couples more strongly (impedance = 1/(2*pi*f*C))". That only holds while the receive node is resistor-dominated. With a ~1 Mohm pulldown and a few pF of pin/wire capacitance, the corner is near 20 kHz **(est.)**. Above it, the link behaves as a capacitive divider (`C_link / (C_link + C_pin)`), which is **flat in frequency**. At 150 kHz the wavelength is about 2 km, so the field is quasi-static and also frequency-flat. There is little physical reason for coupling to ramp across 123-163 kHz. (If no pulldown is fitted, the node floats and the behaviour is even less predictable.)
- The "ramp" the decoder sees is therefore mostly an artifact of its metric. Mean `|x[n] - x[n-1]|` of a sine scales with `sin(pi * f / fs)`. At fs = 500 kS/s that is only about **1.2x** between 123 and 163 kHz, a 20% ramp buried in noise and quantized into 10 bins. That is consistent with `MIN_CORRELATION` having been pushed down to 0.01 in one build.
- Both the Pico (500 kS/s) and ESP (500 kS/s configured) have a Nyquist of 250 kHz, so **the 123-163 kHz tone is directly observable**. A 1 ms Goertzel/FFT resolves 123 vs 163 kHz unambiguously, phase included. The current decoder throws all of that away.
- Rough link budget **(est.)**: plate-to-plate capacitance at 25 cm is perhaps 0.01-0.05 pF versus 5-10 pF at the pin, so a 5 V square gives a few mV to a few tens of mV at the receive node. That is roughly 10-40 LSB on a 12-bit ADC, which is detectable with narrowband processing even though it is poor for a broadband `|delta|` metric.

### A4. The tooling cannot actually see a chirp

- The `s` stream dump returns 10,000 samples on the Pico (20 ms at 500 kS/s) and 1,000 samples on the ESP (2 ms). A chirp lasts 1-2 s. The visualizer shows a 2-20 ms snapshot of a seconds-long event.
- There is no long-record capture, no multi-second spectrogram, and no stored sender-off control recording to compare against.

### A5. No controlled measurement or success criterion

- The sender transmits the constant byte `'a'` forever. A biased or stuck decoder can "succeed" on that. Use random or incrementing payloads plus a CRC, and count frames decoded vs. attempted.
- No false-positive rate is recorded (sender off, same decoder, same duration).
- No sweeps of distance, orientation, ground wiring or frequency have been logged; "about 15 cm" is the only data point.
- Throughput is about **0.1 bit/s**: 36 s preamble + 1 s gap + 27 s data (9 symbols x 3 repeats x 1 s) + 10 s idle = about 74 s per byte. The sender also repeats the same byte every frame, but no receiver soft-combines across frames.
- The sender is autonomous, with no command interface. Although the Nano is on USB serial, nobody can script "emit tone X for 10 s" for a sweep.

### A6. Hidden conductive / common-mode path

- All three boards are on USB to the same host, so their grounds are tied together through the host, and each USB cable is 1-2 m of wire acting as an antenna or counterpoise. A single-ended capacitive link needs a return path, so part of what works at 15 cm may travel via cables and ground rather than plate-to-plate or antenna-to-antenna.
- The Nano's USB-serial chip toggles with every `Serial.print`, and the sender prints during transmission. That is extra correlated switching noise.
- Pico W SMPS: `chirp-receiver` forces PWM mode through WL_GPIO1 (the SMPS power-save pin per the Pico W datasheet). The older `chirp-sdr` writes WL_GPIO2, which as far as I know is VBUS sense, so it probably does nothing. Verify against the datasheet.

### A7. ESP-specific things to verify

- `FAST_ADC_SAMPLE_RATE_HZ = 500000` is an I2S clock setting, not a guarantee of valid conversions. Feed a known tone and check where it lands (aliasing tells you the real rate).
- The ESP32 I2S built-in ADC path is reported by the community to return adjacent sample pairs swapped. That does not matter for an energy average, but it scrambles time order for frequency analysis or sample-to-sample differencing. Verify.
- The ESP32 ADC is nonlinear and noisy. That is fine for tone detection, bad for fine amplitude trends.

---

## Part B. Characterize first, then invent

Do these before building new modulations. They decide which ideas in Part C are worth the time.

**B1. Spur map.** With the sender off, record 10+ s from each receive pin (Pico GP27, GP28; ESP GPIO34) and compute an averaged PSD. Log mains harmonics (50 Hz in Finland), SMPS lines, USB frame energy (1 kHz). Pick sender frequencies that avoid them.

**B2. Tone-SNR sounder.** Have the Nano hold a fixed tone for ~10 s, and have the receiver report power in the tone bin vs. the median of neighbouring bins (Goertzel or long FFT). Sweep OCR2A from 255 (about 31 kHz) to 32 (about 242 kHz) first, which stays below Nyquist, then go above (see C2). Change **one** variable at a time and tabulate SNR in dB:
- distance (5, 10, 15, 25 cm) and plate orientation (face-to-face, edge-on, 90 degrees)
- Pico antenna (GP27) vs. plate (GP28) vs. both
- Nano GND wired to Pico GND on/off
- sender wire attached to D11 vs. D11 toggling with nothing attached

**B3. Which path is the channel?** Run B2 with D11 toggling into nothing. If the tone is still received, the link is via cables/ground. Then reroute the USB cables, swap ports, and power the Nano from the Pico's VBUS pin (USB data to the Nano disconnected) or from a non-host USB source if one is already on hand. Knowing whether the channel is real near-field coupling or the USB cables changes every later decision.

**B4. ESP beacon survey (guaranteed non-zero signal).** Run the ESP SDR over 1-2.5 GHz with the Nano idle-but-powered and list persistent lines: the Nano's 16 MHz crystal comb, the USB-serial chip clock, the Pico's 12 MHz crystal and PLL harmonics, ESP self-spurs. Then change what the Nano is doing (running vs. toggling all pins at max rate vs. held in reset) and diff the spectra. Lines that appear/disappear belong to the Nano, and give you a real detector target and a calibration of how much the ESP can see at 25 cm.

---

## Part C. Ideas and experiments

Roughly ordered by expected payoff per effort. Each has a pass criterion; log results in the template at the bottom.

### C1. Spectral FSK instead of amplitude-trend (cheap, probably the biggest win)
- **Why:** the tone is inside the ADC's Nyquist band, and frequency is a far stronger feature than a 20% amplitude ramp.
- **How:** per-tone Goertzel (or FFT) at the known frequencies. Use many tones, not 17: OCR2A 32..255 gives over 200 distinct frequencies between about 31 and 242 kHz. Start with 4-FSK/8-FSK, 20-100 ms per symbol, and soft-combine repeated frames.
- **Pass:** decode a random payload with CRC at 25 cm in at least 8 of 10 frames, ideally at tens of bit/s instead of 0.1.

### C2. Undersampling as a free mixer (reach the MHz tones)
- **Why:** ADC sampling folds any frequency `f` to `|f - k*fs|`. The sender can go up to 8 MHz (OCR2A = 0) while the ADC still runs at 500 kS/s. MHz tones sit where wire/plate resonances and radiated coupling may be better than at 150 kHz.
- **How:** pick the Nano frequency near a multiple of fs and detune fs (ADC `clkdiv`) so the alias lands at 20-50 kHz, then detect with Goertzel. Odd harmonics of the square wave alias too, so check which alias is strongest. First measure how much of an MHz-range tone the RP2040 and ESP ADC front ends actually pass (analog bandwidth is poorly specified).
- **Pass:** a multi-MHz sender tone detected at 25 cm with SNR better than the 150 kHz tone from B2.

### C3. Better transmitter geometry
- **Differential dipole:** drive D10 and D11 in antiphase so the antenna and the plate form a driven dipole instead of D11 alone. Preload PORTB so one pin is high, then toggle both with a single `out PINB, r` (writing 1s to PINx toggles). Unrolled `out` pairs reach 8 MHz with cycle-exact timing. Pair with a differential receiver (C7).
- **16-bit frequency resolution on D10:** Timer1 CTC mode 12 (TOP = ICR1) can toggle OC1B (D10), giving about 16-bit frequency steps. The sweep-step ISR currently uses Timer1, so it would have to move (cycle-counted steps or Timer0).
- **SPI MOSI = D11:** SPI at fosc/2 shifts arbitrary 8 Mbit/s bit patterns, so you can synthesize tones at arbitrary frequencies (Bresenham / 1-bit sigma-delta style), generate two-tone or phase-flipped carriers, and generate gated bursts.
- **Port-wide toggling:** toggling whole ports (`PORTx = 0xFF/0x00`) maximizes supply di/dt and radiates from the board and USB cable. Crude but worth one test.
- **Output state:** compare D11 driven, D11 as Hi-Z input, and D11 driven low, as a quick map of what the plate actually does.

### C4. Phase modulation once SNR is known
- **Why:** 123-250 kHz carriers are within the receive Nyquist band, so phase is measurable. The Nano and Pico crystals differ by tens of ppm, which is only a few Hz of carrier offset at 150 kHz.
- **How:** differential BPSK (invert the pattern or shift phase by half a period between symbols; decode by comparing consecutive symbols so no carrier recovery loop is needed), or simple OOK as a stepping stone. Needs a sender whose phase can be controlled (cycle-counted loops or SPI patterns from C3).
- **Pass:** hundreds of bit/s at a BER you can measure.

### C5. Passive capacitive load modulation (RFID-style, no transmit power)
- **Why:** the plates are a real capacitor. If D11 toggles between output-low (grounded plate, like a finger) and Hi-Z (floating), the Pico plate's self-capacitance changes slightly, which is exactly what capacitive touch sensing detects.
- **How (Pico):** measure GP28's RC charge time with a PIO loop (drive low, release with pull-up, count PIO cycles to the logic threshold). Average many thousands of cycles per reading, then lock in on the Nano's known toggle pattern at 5-50 Hz. Expected dC is maybe 0.01-0.05 pF **(est.)**, which is a sub-nanosecond timing shift; jitter acts as free dither.
- **How (ESP32):** the ESP32 has a dedicated touch peripheral. Move the jumper to GPIO32 or GPIO33 (T9/T8) and run the touch sensor with heavy filtering against the same Nano pattern.
- **Pass:** a repeatable 1-10 bit/s link at 25 cm that survives the sender being powered from a different supply (B3).

### C6. Heterodyne by pin nonlinearity (Pico LO injection)
- **Why:** an input pin's ESD diodes and the ADC sampling switch are nonlinear, so they mix.
- **How:** output a square wave from a spare Pico pin near the receive node (PIO/PWM, kHz to tens of MHz) and tune it so `|f_nano - f_LO|` lands in the ADC band. Combine with C2 to choose which Nano harmonic you convert down.
- **Pass:** a tone that tracks the LO offset when you step the LO frequency.

### C7. Receiver upgrades that need no new parts
- **Differential / diversity on the Pico:** `adc_set_round_robin(0b110)` samples GP27 (antenna) and GP28 (plate) interleaved at fs/2 each. Subtracting them cancels common noise (SMPS, mains, USB) while the signal differs between the two. Also try them added.
- **Long raw captures:** DMA 100k+ samples (pack to 8 bits if needed) into RAM for offline analysis, or compute band energies and spectrogram rows on-device and stream those instead of raw samples. This fixes A4.
- **1-bit PIO sampler:** a GPIO Schmitt input sampled by PIO at 125-250 MS/s sees up to 62-125 MHz with no aliasing concerns. Bias the node near the switching threshold (briefly drive the pin, then float it, or dither using the neighbouring pin) so tiny coupled swings flip bits. Count transitions or correlate with the known waveform.
- **Pico + ESP diversity:** run a jumper from a Pico GPIO to an ESP GPIO as a shared time marker, record the same transmission on both, and soft-combine. Independent receiver noise averages down.

### C8. ESP32 raw-radio SDR (the harmonic hunt)
- **Frequency uncertainty:** the Nano's crystal is not locked to the ESP. Harmonic n is off by roughly n x ppm: +/-20 ppm at 1 GHz is +/-20 kHz, plus drift. Do **not** assume a fixed bin. Detect energy in a +/-100 kHz window, track the peak, then integrate.
- **Narrowband integration:** a CW line can be detected far below WiFi packet sensitivity. A 1 kHz resolution bandwidth has a thermal floor around -135 dBm after noise figure **(est.)**, versus roughly -90 dBm for decoding packets, so seconds of averaging buys tens of dB.
- **OOK on a harmonic or clock comb:** gate the toggle (connect/disconnect OC2A via COM2A0), or alternate "all pins toggling at max rate" with "idle". One bit per 0.5-2 s with long integration is the realistic first target.
- **Run the existing sender unchanged:** its fundamental sweeps 123-163 kHz, so the nth harmonic sweeps n times wider (about 280 MHz at n around 7000, near 1 GHz). Look at the ESP SDR spectrogram for slanted lines. The slope sign is UP vs. DOWN, so the current protocol might decode on the ESP without changing the sender.
- **Calibrate with a known beacon:** drive a PIO square wave at 50-125 MHz from the Pico (GP27 antenna or GP28 plate) and see what the ESP detects at 25 cm. That brackets what a much slower, 8x-lower-fundamental AVR edge can possibly do. The Pico W's CYW43 2.4 GHz transmitter also gives an in-band reference for the receive chain.
- **Impulse / edge PPM:** a single edge from a 5 V port is broadband. Encode data in inter-edge intervals (62.5 ns granularity with cycle-counted `out` instructions) and look for wideband bursts in the ESP's I/Q time domain, instead of hunting one spectral line.
- **EM activity keying:** alternate high-activity code (all ports toggling, ADC/UART/timers on) with the quietest state you can reach, and treat integrated band energy over a wide window as the "carrier".

### C9. Backscatter with the D10 antenna (2.4 GHz without generating 2.4 GHz)
- **Why:** the Nano cannot make 2.4 GHz, but it can change what its antenna does to 2.4 GHz that something else already sends. At 25 cm, a 20 dBm WiFi transmitter delivers roughly -8 dBm to the antenna **(est.: 20 dBm minus about 28 dB free-space loss)**.
- **How:** switch D10 between output-low (shorted), Hi-Z (open) and output-high at 10-500 Hz. Illuminate with the Pico W or ESP32 sending continuous UDP traffic on a fixed channel. Detect the change at the other radio via ESP32 CSI (`esp_wifi_set_csi_*`: per-subcarrier amplitude and phase per packet) or via the I/Q envelope in the SDR, and lock in at the modulation rate. Try ambient router traffic as illumination too.
- **Free sync/downlink check:** at that power level, a WiFi burst may be rectified by the Nano pin's protection diodes. Try D10 as a digital input and see whether bursts flip it. If so, you get a wake-up or sync channel that would replace the 36 s preamble.
- **Pass:** a CSI or envelope modulation that follows the Nano's toggle pattern and disappears when D10 is disconnected.

### C10. Protocol hygiene (after C1-C9 show which channel works)
- Random payload + length + CRC; log decoded/attempted frames and the sender-off false-positive rate.
- Replace the 36 s preamble with a short sync word of strong autocorrelation (e.g. Barker-like) once there is SNR to spare.
- Soft-combine across repeated frames: the sender already repeats the same message, which gives free processing gain.
- Measure BER vs. symbol time to find the real capacity, then step the rate up until BER rises.
- Add a command interface on the Nano (`tone <ocr> <ms>`, `off`, `pattern ...`) so sweeps in B2/C-experiments can be scripted from the host. Keep the serial port quiet during transmission (A6).

### C11. Environment and repeatability
- Fix the boards on a ruler or tape marks: a 1 cm shift can change coupling by tens of percent, which will swamp every other effect.
- Keep hands, laptops, chargers and lamps away during runs; record what else is on the desk.
- Repeat each measurement at least 3 times and interleave sender-off controls.

---

## Results log template

Copy per experiment:

```
### <date> <idea id, e.g. C1>
Setup: distance, orientation, which pins, ground wired?, USB power source
Sender: pin(s), frequency/pattern, duration
Receiver: board, pin, fs, detector, integration time
Result: SNR (dB) or BER (n errors / n bits), frames decoded / attempted
Control: same run with sender off -> false-positive rate / noise floor
Conclusion + next step:
```
