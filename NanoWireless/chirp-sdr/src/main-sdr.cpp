/*
 * CHIRP RECEIVER -- Raspberry Pi Pico W port
 * -------------------------------------------
 * Reads ADC0 (GP26 / labeled "A0") and decodes UP/DOWN chirps into bits,
 * using the same capacitive-coupling amplitude-trend technique as the
 * original AVR Nano receiver. Logic is unchanged; only the platform-specific
 * bits (ADC setup/resolution, pin naming, build config) differ.
 *
 * PICO-SPECIFIC NOTES:
 *   - Wire the antenna stub / pickup wire to GP26 (silkscreened "A0" on
 *     most Pico boards), and share GND with the sender.
 *   - The Pico's onboard ADC is native 12-bit (0-4095), vs. the Nano's
 *     10-bit (0-1023). We call analogReadResolution(12) explicitly so we
 *     get the full native resolution -- this is a free sensitivity win for
 *     the mean-abs-delta activity metric (finer steps = more granularity
 *     to detect weak signal-driven jitter above noise).
 *   - analogRead() timing on RP2040 is roughly similar in the ballpark of
 *     the AVR's (tens of microseconds), but not identical -- if you need
 *     precise inter-sample timing guarantees, verify empirically. For this
 *     receiver's purposes (averaged activity metrics, not exact sample
 *     rate) that variance doesn't matter.
 *   - Serial here is the Pico's native USB CDC serial (not a hardware
 *     UART). monitor_speed / Serial.begin's baud argument is effectively
 *     ignored by USB CDC but kept for source compatibility.
 *   - millis()/micros()/F() all behave the same as AVR Arduino under the
 *     arduino-pico core, so all the timing/logging code is unchanged.
 *
 * The ADC on this board still cannot directly sample a multi-MHz chirp --
 * so instead of measuring frequency, we exploit the physics of near-field
 * capacitive coupling: impedance = 1/(2*pi*f*C), so a HIGHER frequency
 * couples MORE strongly into the ADC pin (bigger apparent amplitude/
 * jitter), and a LOWER frequency couples more weakly. So:
 *   - an UP chirp (low->high freq over the second)   => amplitude rises
 *   - a DOWN chirp (high->low freq over the second)  => amplitude falls
 * We slice each individual chirp window into 10 bins, compute an
 * "activity" metric (mean |sample-to-sample delta|) per bin, then correlate
 * the resulting 10-point profile against an ideal rising-ramp template
 * (Pearson correlation coefficient) to decide UP vs DOWN -- a simple
 * matched filter that uses every bin instead of just comparing a couple of
 * edge averages. See the detailed comment on sampleChirpBit() for why.
 *
 * SYMBOL REDUNDANCY (matches sender):
 *   Each logical bit is transmitted as a "symbol" of SYMBOL_REPEATS
 *   back-to-back identical chirps (all UP or all DOWN), not a single
 *   chirp. We decode each of the sub-chirps independently with
 *   sampleChirpBit() and majority-vote the result for the symbol -- this
 *   makes bit decisions robust to a single bad/noisy sub-chirp. Sub-chirp
 *   restart edges (OCR2A snapping back to its start value on the sender)
 *   do produce a brief transient right at the start of each sub-chirp,
 *   which can slightly pollute that sub-chirp's "early" bins -- majority
 *   voting across repeats means a single polluted sub-chirp normally can't
 *   flip the symbol's decoded bit.
 *
 * HARDWARE NOTE:
 *   A floating ADC pin picks up ambient noise (mains hum, nearby wires)
 *   that can swing wildly and falsely look like a signal. Add a ~1M ohm
 *   resistor from GP26 to GND to give it a stable baseline when nothing is
 *   coupling in.
 *
 * CALIBRATION:
 *   At startup the receiver measures the idle noise floor on the ADC pin
 *   for ~2s and sets adcActiveThreshold dynamically from that (mean + 5
 *   std devs, with a floor of MIN_ACTIVITY_THRESHOLD -- widened from 3
 *   std devs / a lower floor because ambient noise bursts were routinely
 *   crossing the old threshold and faking a preamble onset). Keep
 *   everything quiet (sender off / pickup wire not near the sender wire)
 *   while it boots -- the calibration is only as good as how quiet this
 *   window actually is. Onset detection also requires the signal to stay
 *   above threshold for ONSET_DEBOUNCE (now 8) consecutive checks, so a
 *   brief noise burst can't fake a frame start. MIN_CORRELATION controls
 *   bit-decision confidence -- if your log shows |r| values only just
 *   above 0 for real chirps, lower this; if ambiguous calls are frequently
 *   wrong, raise it.
 *
 * ONSET DETECTION (mean-abs-delta, not peak-to-peak):
 *   Onset detection uses the same "mean absolute sample-to-sample delta"
 *   metric as bit decoding, averaged over many samples, rather than a raw
 *   peak-to-peak swing over a short window. Peak-to-peak is dominated by
 *   rare outlier samples (a single noise spike can spike min/max just as
 *   much as real signal), which forces the threshold high and hides weak
 *   real signals. Averaging a delta metric over hundreds of samples lets
 *   the law of large numbers work in our favor: a real periodic chirp
 *   raises the average consistently, even when no single sample-to-sample
 *   delta is individually distinguishable from noise.
 *
 * Protocol expected (ALL of these are configurable constants below; whatever
 * you set them to here must be mirrored exactly in the sender's copy):
 *   [idle]
 *   -> [preamble: PREAMBLE_LEN symbols @ PREAMBLE_MS/sub-chirp x SYMBOL_REPEATS
 *                 repeats, pattern EXPECTED_PREAMBLE[]]
 *   -> [PREAMBLE_GAP_MS idle]
 *   -> [message: 8 data bits + 1 parity bit, each a symbol @ DATA_MS/sub-chirp
 *                x SYMBOL_REPEATS repeats, MSB first]
 *   -> [idle] -> repeat
 */

#define PICO_FLASH_SPI_CLKDIV 4
#include <Arduino.h>
#include "hardware/adc.h"
#include "pico/cyw43_arch.h" // needed for the SMPS PWM-mode GPIO on Pico W

// ---- Debug/streaming support ----
// Typing 's' + Enter on the serial console dumps STREAM_SAMPLE_COUNT raw
// ADC samples as binary uint16 (little-endian) instead of running the
// decoder, so an external tool (e.g. the companion Python visualizer) can
// plot the waveform/spectrum/correlation live without the firmware itself
// doing any FFT work. Framed as: 4 sync bytes 0xAA 0x55 0xAA 0x55, then a
// uint32 sample count, then the raw uint16 samples, then 0xFF x4 as an
// end marker. Decode logic (the actual receiver) is untouched -- this is
// purely a side debug channel invoked by the operator.
static const uint32_t STREAM_SAMPLE_COUNT = 10000; // ~ a couple of chirp windows worth

static const uint8_t ADC_PIN = A0;             // GP26 on Pico W
static const uint8_t ADC_RESOLUTION_BITS = 12; // native RP2040 ADC resolution
static const uint32_t PREAMBLE_MS = 2000;      // duration of each sub-chirp in a preamble symbol
static const uint32_t DATA_MS = 1000;          // duration of each sub-chirp in a data symbol (must match sender's DATA_MS)
static const uint8_t SYMBOL_REPEATS = 1;       // sub-chirps per symbol (must match sender)
static const uint32_t PREAMBLE_GAP_MS = 1500;  // idle gap between preamble and message (matches sender)
static const uint8_t BINS = 10;
static const float MIN_CORRELATION = 0.2f;        // TUNE THIS -- Pearson r magnitude below this is flagged [ambiguous]
static const float MIN_ACTIVITY_THRESHOLD = 1.6f; // floor for auto-calibrated onset threshold (raised from 0.8 -- was letting ambient noise trigger onset)
static const uint8_t ONSET_DEBOUNCE = 8;          // consecutive onset scans required (raised from 4 -- makes brief noise bursts far less likely to fake an onset)
static const uint16_t ONSET_SAMPLES = 500;        // averaging window for onset activity metric

// ---- Protocol config (EVERYTHING here must match the sender's copy) ----

// Must be byte-for-byte identical to the sender's PREAMBLE_PATTERN[].
// Length is derived automatically -- just edit the list.
static const bool EXPECTED_PREAMBLE[] = {true, true, true, true, false, false};
static const uint8_t PREAMBLE_LEN = sizeof(EXPECTED_PREAMBLE) / sizeof(EXPECTED_PREAMBLE[0]);

float adcActiveThreshold = 1.0f; // overwritten by calibrateNoiseFloor() at startup

// ---- SMPS mode ----
// The Pico W's onboard switcher defaults to PFM ("power save") mode at
// light load, which drops in and out of a low-frequency switching pattern
// -- that switching noise/ripple lands right in-band for a sensitive
// capacitive-coupling ADC pickup. Forcing PWM ("improved ripple") mode
// keeps the switcher running at a constant, higher frequency instead, so
// its noise is fixed and easy to filter/ignore rather than an unpredictable
// broadband burst every time load changes. On Pico W this pin isn't a
// plain GPIO -- it's wired through the CYW43 wireless chip, so it must be
// set via cyw43_arch, which means bringing that driver up even though we
// otherwise don't use WiFi at all here.
void setSmpsPwmMode()
{
  // NOTE: do NOT call cyw43_arch_init() here -- arduino-pico's Pico W core
  // already initializes the CYW43 driver during its own boot sequence
  // (it needs it just to drive the onboard LED). Calling it again here
  // re-enters/deadlocks the driver *before* Serial.begin() gets a chance
  // to bring up USB CDC, which hangs the board with no serial output at
  // all and breaks the normal USB auto-reset-into-bootloader path (hence
  // needing to hold BOOTSEL to reprogram). Just use the GPIO directly.
  const uint8_t CYW43_SMPS_MODE_GPIO = 2;
  cyw43_arch_gpio_put(CYW43_SMPS_MODE_GPIO, 1);
  Serial.println(F("[ok] SMPS forced to PWM mode (low ripple)"));
}

// ---- Fast ADC (hardware FIFO, free-running) ----
// analogRead() re-does the whole ADC init/select/start/wait dance on every
// single call, which is slow (tens of us of overhead on top of the actual
// conversion) and -- worse for an "SDR-like" receiver -- jittery, since
// that overhead isn't constant. Configuring the ADC to free-run into its
// hardware FIFO instead gives back-to-back conversions at a fixed rate
// with no software involved in between, so consecutive samples are evenly
// spaced. We still pull samples out in software (no DMA) to keep this
// simple, but each read is now just a FIFO pop instead of a full ADC cycle.
static const uint32_t FAST_ADC_SAMPLE_RATE_HZ = 500000; // RP2040 ADC clock is 48MHz; clkdiv sets the divide

void setupFastAdc()
{
  adc_init();
  adc_gpio_init(26); // GP26 / A0
  adc_select_input(0);
  adc_fifo_setup(
      true,  // write conversions to FIFO
      false, // no DMA request signalling (we poll)
      1,     // assert IRQ/flag once 1 sample is in the FIFO
      false, // no error bit in FIFO entries
      false  // don't right-shift to 8 bit -- keep full 12-bit samples
  );
  float clkdiv = (48000000.0f / FAST_ADC_SAMPLE_RATE_HZ) - 1.0f;
  adc_set_clkdiv(clkdiv);
  adc_fifo_drain();
  adc_run(true); // free-run: keep converting continuously into the FIFO
}

// Pops the oldest sample out of the ADC FIFO, blocking briefly if the FIFO
// is momentarily empty. Drop-in replacement for analogRead(ADC_PIN) used
// everywhere below, but backed by the free-running FIFO above instead of
// a fresh conversion each call.
inline uint16_t fastAdcRead()
{
  while (adc_fifo_is_empty())
  { /* wait for next free-running conversion */
  }
  return adc_fifo_get();
}

// Streams STREAM_SAMPLE_COUNT raw samples out over Serial as a simple
// binary frame for the companion Python visualizer. Pure debug utility --
// does not touch adcActiveThreshold, decode state, or anything the
// decoder relies on.
void streamRawAdc()
{
  Serial.println(F("[stream] starting raw ADC dump..."));
  uint8_t sync[4] = {0xAA, 0x55, 0xAA, 0x55};
  Serial.write(sync, 4);
  uint32_t count = STREAM_SAMPLE_COUNT;
  Serial.write((uint8_t *)&count, 4);
  for (uint32_t i = 0; i < count; i++)
  {
    uint16_t v = fastAdcRead();
    Serial.write((uint8_t *)&v, 2);
  }
  uint8_t endMarker[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  Serial.write(endMarker, 4);
  Serial.flush();
  Serial.println(F("[stream] done"));
}

// Mean absolute sample-to-sample delta over N reads -- same metric
// sampleChirpBit() uses per-bin for bit decisions. Far more robust than
// peak-to-peak: a lone noise spike barely moves an average over hundreds
// of samples, whereas a real periodic chirp raises it consistently.
float measureActivity(uint16_t samples)
{
  long sum = 0;
  int last = fastAdcRead();
  for (uint16_t i = 0; i < samples; i++)
  {
    int v = fastAdcRead();
    sum += abs(v - last);
    last = v;
  }
  return (float)sum / samples;
}

// Measures idle activity on the ADC pin over ~2s (mean + a margin based on
// observed variability, not a raw peak-to-peak reading) and sets
// adcActiveThreshold comfortably above it. Call this while the sender is
// OFF / out of range.
void calibrateNoiseFloor()
{
  Serial.println(F("[cal] measuring idle noise floor for 2s -- keep quiet..."));
  float sumActivity = 0;
  float sumSqActivity = 0;
  uint16_t n = 0;
  uint32_t start = millis();
  while (millis() - start < 2000)
  {
    float a = measureActivity(ONSET_SAMPLES);
    sumActivity += a;
    sumSqActivity += a * a;
    n++;
  }
  float mean = sumActivity / n;
  float variance = (sumSqActivity / n) - (mean * mean);
  float stddev = variance > 0 ? sqrt(variance) : 0;

  // mean + 5 std devs: a margin against noise *variability*, not a blanket
  // multiplier against noise *peak*. Widened from 3 to 5 std devs because
  // 3 was letting ordinary ambient noise bursts (mains hum, nearby wires,
  // USB noise) cross the threshold often enough to fake a preamble onset.
  adcActiveThreshold = max(mean + 5.0f * stddev, MIN_ACTIVITY_THRESHOLD);
  Serial.print(F("[cal] idle activity mean="));
  Serial.print(mean, 3);
  Serial.print(F(" stddev="));
  Serial.print(stddev, 3);
  Serial.print(F(" -> threshold set to "));
  Serial.println(adcActiveThreshold, 3);
}

// Scans the ADC pin briefly and returns true if activity suggests a signal
// is present (vs. quiet baseline noise).
bool detectSignalOnsetOnce()
{
  float activity = measureActivity(ONSET_SAMPLES);
  return activity > adcActiveThreshold;
}

// Debounced version: requires ONSET_DEBOUNCE consecutive positive scans
// before declaring a real onset, to reject brief noise spikes.
bool detectSignalOnset()
{
  for (uint8_t i = 0; i < ONSET_DEBOUNCE; i++)
  {
    if (!detectSignalOnsetOnce())
      return false;
  }
  return true;
}

// Samples one *single* chirp window (aligned to windowStart, in millis())
// of the given duration, and returns true if amplitude activity correlates
// with a rising ramp (bit=1, UP chirp), false if it correlates with a
// falling ramp (bit=0, DOWN chirp). Prints an [ambiguous] tag when the
// correlation magnitude is within the noise-level MIN_CORRELATION gate, so
// you can see when a call was a coin flip.
//
// MATCHED-FILTER / CORRELATION DECODING:
//   The old approach only compared the average of the first 3 bins against
//   the last 3, throwing away the middle 4 bins entirely. That's wasteful --
//   every sample carries some signal, even the noisier middle ones -- and
//   it means a single noisy edge bin can flip the decision outright.
//   Instead, we correlate the whole 10-bin activity profile against an
//   ideal monotonic ramp template using the Pearson correlation
//   coefficient r. This is a simple matched filter: it asks "how well does
//   this whole shape match a rising trend?" rather than "is the end bigger
//   than the start?" Using every sample point (not just 6 of 10) is the
//   same principle behind LoRa's chirp correlation and behind
//   processing/spreading gain generally -- integrating more samples against
//   a known reference shape pulls a weak, noisy signal's trend out more
//   reliably than comparing a couple of raw averages. r ranges from -1
//   (perfect falling ramp) to +1 (perfect rising ramp); r near 0 means the
//   profile didn't look like either and the call is a coin flip.
bool sampleChirpBit(uint32_t windowStart, uint32_t durationMs)
{
  float binActivity[BINS];
  uint32_t binMs = durationMs / BINS;

  for (uint8_t b = 0; b < BINS; b++)
  {
    uint32_t binStart = windowStart + (uint32_t)b * binMs;
    uint32_t binEnd = binStart + binMs;

    // wait until this bin actually starts (keeps us aligned to the sender's clock)
    while ((int32_t)(millis() - binStart) < 0)
    {
    }

    long sum = 0;
    uint16_t count = 0;
    int last = fastAdcRead();
    while ((int32_t)(millis() - binEnd) < 0)
    {
      int v = fastAdcRead();
      sum += abs(v - last);
      last = v;
      count++;
    }
    binActivity[b] = count > 0 ? (float)sum / count : 0.0f;
  }

  // Ideal rising-ramp template: 0, 1, 2, ... BINS-1. A DOWN chirp's profile
  // should correlate negatively against this same template (falling ramp is
  // just the rising ramp's mirror image), so one template covers both cases
  // via the sign of r.
  float meanActivity = 0;
  for (uint8_t b = 0; b < BINS; b++)
    meanActivity += binActivity[b];
  meanActivity /= BINS;

  const float meanTemplate = (BINS - 1) / 2.0f;

  float covariance = 0, varActivity = 0, varTemplate = 0;
  for (uint8_t b = 0; b < BINS; b++)
  {
    float da = binActivity[b] - meanActivity;
    float dt = (float)b - meanTemplate;
    covariance += da * dt;
    varActivity += da * da;
    varTemplate += dt * dt;
  }

  float denom = sqrt(varActivity * varTemplate);
  float r = (denom > 1e-6f) ? (covariance / denom) : 0.0f;

  Serial.print(F("    activity/bin: "));
  for (uint8_t b = 0; b < BINS; b++)
  {
    Serial.print(binActivity[b], 1);
    Serial.print(' ');
  }
  Serial.print(F(" | r="));
  Serial.print(r, 3);
  if (fabs(r) < MIN_CORRELATION)
    Serial.print(F(" [ambiguous]"));
  Serial.println();

  return r > 0; // positive correlation with rising ramp => UP chirp => bit 1
}

// Samples an entire *symbol* -- SYMBOL_REPEATS back-to-back sub-chirps of
// chirpMs each, all carrying the same bit -- and returns the majority-voted
// bit. This matches the sender's redundant symbol scheme and makes the
// decision robust to a single noisy/glitched sub-chirp.
bool sampleSymbolBit(uint32_t symbolStart, uint32_t chirpMs)
{
  uint8_t votes = 0;
  for (uint8_t r = 0; r < SYMBOL_REPEATS; r++)
  {
    uint32_t subStart = symbolStart + (uint32_t)r * chirpMs;
    bool bit = sampleChirpBit(subStart, chirpMs);
    if (bit)
      votes++;
  }
  bool result = votes > (SYMBOL_REPEATS / 2);
  Serial.print(F("  symbol vote: "));
  Serial.print(votes);
  Serial.print(F("/"));
  Serial.print(SYMBOL_REPEATS);
  Serial.print(F(" -> bit="));
  Serial.println(result ? 1 : 0);
  return result;
}

// ---- Frame-level logic ----

void setup()
{
  Serial.begin(115200);
  // Pico W's Serial is native USB CDC. Give the host a few seconds to
  // enumerate/open the port so early prints aren't lost, but don't hang
  // forever if nothing's connected (e.g. running off USB power only).
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000)
  { /* wait for USB CDC */
  }
  delay(200);

  setSmpsPwmMode(); // low-ripple switcher mode, before we start relying on a clean ADC
  setupFastAdc();   // free-running FIFO ADC, replaces analogRead() everywhere below

  Serial.println(F("=== Chirp Receiver ready (Pico W) ==="));
  Serial.println(F("[hint] type 's' + Enter any time to dump a raw ADC stream for the Python visualizer"));
  calibrateNoiseFloor();
  Serial.println(F("Waiting for quiet baseline, then listening for a frame..."));
}

// Non-blocking check for the 's' stream-dump command. Safe to call from
// inside spin/wait loops (unlike doing this only once per outer loop()
// iteration) so the debug stream stays responsive even while the receiver
// is sitting in a long "waiting for quiet"/"waiting for onset" spin --
// those can block for seconds or indefinitely if a sender is transmitting
// continuously, and we don't want a debug request to have to wait that
// long. Returns true if a stream was just sent.
bool checkStreamCommand()
{
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == 's' || c == 'S')
    {
      streamRawAdc();
      return true;
    }
  }
  return false;
}

void loop()
{
  // Debug hook: if the operator typed 's', dump a raw ADC stream instead of
  // running the normal sync/decode cycle this iteration.
  if (checkStreamCommand())
  {
    return;
  }

  // Wait for a clean idle period first, so we don't sync mid-frame
  Serial.println(F("[sync] waiting for quiet..."));
  while (detectSignalOnset())
  {
    checkStreamCommand(); // stays responsive during a possibly-long/indefinite wait
  }

  Serial.println(F("[sync] quiet detected, listening for onset..."));
  while (!detectSignalOnset())
  {
    checkStreamCommand();
  }

  uint32_t t0 = millis();
  Serial.println(F("[sync] signal onset -> frame start (t0)"));

  uint32_t preambleSymbolMs = PREAMBLE_MS * SYMBOL_REPEATS;
  uint32_t dataSymbolMs = DATA_MS * SYMBOL_REPEATS;

  // --- Preamble: PREAMBLE_LEN symbols, pattern from EXPECTED_PREAMBLE[] ---
  bool preambleOK = true;

  Serial.print(F("--- decoding preamble ("));
  Serial.print(PREAMBLE_LEN);
  Serial.println(F(" symbols) ---"));
  for (uint8_t i = 0; i < PREAMBLE_LEN; i++)
  {
    uint32_t symbolStart = t0 + (uint32_t)i * preambleSymbolMs;
    bool bit = sampleSymbolBit(symbolStart, PREAMBLE_MS);
    Serial.print(F("  preamble symbol "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(bit ? 1 : 0);
    if (bit != EXPECTED_PREAMBLE[i])
    {
      // Bail immediately instead of decoding all 6 symbols -- a false onset
      // (noise, not a real chirp) is extremely unlikely to also produce a
      // matching first bit, so this is the common case and catching it
      // early saves several seconds of blind decode time per false trigger,
      // getting back to listening for a real onset that much sooner.
      preambleOK = false;
      Serial.print(F("[!] preamble symbol "));
      Serial.print(i);
      Serial.println(F(" mismatch -- aborting preamble decode, resyncing"));
      break;
    }
  }

  if (!preambleOK)
  {
    return;
  }
  Serial.println(F("[ok] preamble matched!"));

  // --- gap, then 8 data bits + 1 parity bit, each a symbol @ 500ms/sub-chirp x4, MSB first ---
  uint32_t messageStart = t0 + (uint32_t)PREAMBLE_LEN * preambleSymbolMs + PREAMBLE_GAP_MS;
  uint8_t byteVal = 0;

  Serial.println(F("--- decoding message (500ms/sub-chirp x4) ---"));
  for (uint8_t i = 0; i < 8; i++)
  {
    uint32_t symbolStart = messageStart + (uint32_t)i * dataSymbolMs;
    bool bit = sampleSymbolBit(symbolStart, DATA_MS);
    byteVal = (byteVal << 1) | (bit ? 1 : 0);
    Serial.print(F("  data bit "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(bit ? 1 : 0);
  }

  uint32_t paritySymbolStart = messageStart + 8UL * dataSymbolMs;
  bool receivedParity = sampleSymbolBit(paritySymbolStart, DATA_MS);
  Serial.print(F("  parity bit: "));
  Serial.println(receivedParity ? 1 : 0);

  // recompute expected even parity from the decoded byte
  uint8_t expectedParity = 0;
  for (uint8_t i = 0; i < 8; i++)
    expectedParity ^= (byteVal >> i) & 0x01;
  bool parityOK = (receivedParity == (bool)expectedParity);

  Serial.print(F("=== DECODED BYTE: 0x"));
  Serial.print(byteVal, HEX);
  Serial.print(F(" = '"));
  Serial.write(byteVal);
  Serial.print(F("' | parity "));
  Serial.println(parityOK ? F("OK ===") : F("MISMATCH -- likely bit error! ==="));
}