/*
 * CHIRP SENDER
 * ------------
 * Generates a swept square wave ("chirp") on pin D11 (Timer2 / OC2A)
 * using hardware CTC toggle mode. This is a near-field / capacitive-coupling
 * transmitter, not a real RF radio -- keep the receiver's antenna wire
 * within a few cm of this pin's wire.
 *
 * Frequency range (Timer2, NO prescaler, 16 MHz clock):
 *   f = F_CPU / (2 * (OCR2A + 1))
 *   OCR2A = 50  -> ~156.9 kHz  (lowest)
 *   OCR2A = 0   -> 8 MHz       (highest, = F_CPU/2, the physical max)
 *
 * NOTE: the receiver decodes bits by *amplitude trend*, not by measuring
 * frequency directly (it never calls pulseIn() or counts pulse widths) --
 * it exploits the fact that near-field capacitive coupling impedance is
 * 1/(2*pi*f*C), so HIGHER frequency couples MORE strongly into the
 * receiver's ADC pin (bigger sample-to-sample jitter), and lower frequency
 * couples more weakly. A prior revision of this sweep dropped to
 * 500Hz-10.4kHz under the mistaken assumption the receiver needed
 * individually-measurable pulses -- that range is actively counterproductive
 * for an amplitude-trend receiver, since it keeps coupling weak the entire
 * chirp and starves the receiver's signal-to-noise margin. This version
 * keeps the whole sweep in the strong-coupling high-frequency band instead.
 *
 * SPREADING (LoRa-inspired processing gain):
 *   Each bit is sent as REPEATS identical back-to-back chirps instead of
 *   just one. The receiver treats these as one "symbol" and integrates
 *   (matched-filter correlates + sums) across all REPEATS of them before
 *   deciding the bit. This trades data rate for noise immunity, same
 *   tradeoff LoRa's spreading factor makes. Bump REPEATS up if you need
 *   more range/reliability, down for more speed.
 *
 * Protocol (matches receiver -- ALL of these are now configurable constants
 * in the "Protocol config" block below; whatever you set them to here must
 * be mirrored exactly in the receiver's copy of the same constants):
 *   [IDLE_MS idle/silence]  <-- lets receiver detect frame onset
 *   [preamble: PREAMBLE_LEN symbols, pattern PREAMBLE_PATTERN[]]
 *       each symbol = REPEATS x PREAMBLE_MS chirps
 *   [PREAMBLE_GAP_MS idle/silence]
 *   [message: 8 data bits + 1 parity bit]   each symbol = REPEATS x DATA_MS chirps
 *       MSB first, byte = MESSAGE_BYTE, parity = even parity of those 8 bits
 *   [IDLE_MS idle/silence]
 *   ... repeat forever
 *
 * Wire D11 to a short stub of wire (a few cm) to act as a crude antenna.
 * Share a common GND between the two Nanos -- without it, capacitive
 * pickup on the receiver will be weak/unstable.
 *
 * PERFORMANCE NOTES (AVR side):
 *   - CHIRP_PIN's setup/idle-low is now done with direct DDRB/PORTB writes
 *     instead of pinMode()/digitalWrite(), which each cost a table lookup
 *     under the hood. This hardcodes D11 == PB3 (true for the ATmega328
 *     Nano) -- if you ever change CHIRP_PIN off pin 11, update
 *     chirpPinSetup()/chirpPinIdleLow() by hand, since the fast path no
 *     longer looks the pin up generically.
 *   - The per-step frequency sweep is driven by a Timer1 CTC compare-match
 *     interrupt (see startStepTimer()/ISR(TIMER1_COMPA_vect)) instead of a
 *     main-loop busy-wait polling micros(). This matters because a polled
 *     micros() busy-wait can only notice "it's time for the next step"
 *     sometime after the deadline, by however long the poll's own loop
 *     body + micros() call takes -- a real but small per-step slip that
 *     accumulates over 256 steps. A hardware compare-match interrupt fires
 *     at the exact clock edge instead, so step timing tracks the crystal
 *     directly rather than tracking "main loop polling latency".
 *   - The final "pad out to exactly durationMs" wait uses a coarse
 *     micros()-polled wait down to the last ~50us, then a hand-written
 *     inline-assembly cycle-counted delay (delayCyclesAsm()) for the
 *     remainder -- see the comment above that function for why.
 *   - See platformio.ini (if using PlatformIO) for -O3/-flto build flags.
 *     In the Arduino IDE, the stock avr-gcc toolchain defaults to -Os
 *     (size, not speed); there's no per-sketch UI toggle for this, so if
 *     you want -O3 there you'd need to edit
 *     hardware/tools/avr/etc/avr8-gnu-toolchain/.../platform.txt (or
 *     switch to PlatformIO / arduino-cli, which both accept build_flags
 *     cleanly). None of this sketch's logic depends on it -- it's a
 *     compiler-level speed/size tradeoff, not a correctness requirement.
 */

#include <Arduino.h>
#include <avr/interrupt.h>

static const uint8_t CHIRP_PIN = 11;      // OC2A
static const uint16_t STEPS = 256;        // sweep resolution
static const uint32_t PREAMBLE_MS = 2000; // one preamble chirp: 1 second
static const uint32_t DATA_MS = 1000;     // one data/parity chirp: 0.5 second (must match receiver's DATA_MS)
static const uint8_t OCR_LOW = 64;        // ~156.9 kHz (lowest freq -- still strong coupling)
static const uint8_t OCR_HIGH = 48;       // 8 MHz      (highest freq, physical max)
static const uint8_t REPEATS = 3;         // chirps per bit (spreading factor, must match receiver's SYMBOL_REPEATS)

// ---- Protocol config (EVERYTHING here must match the receiver's copy) ----

// Preamble pattern: any length, any mix of UP(1)/DOWN(0) symbols you want.
// The array length is derived automatically (PREAMBLE_LEN below) -- just
// edit the list. Must be identical to the receiver's expectedPreamble[].
static const bool PREAMBLE_PATTERN[] = {true, true, true, true, false, false};
static const uint8_t PREAMBLE_LEN = sizeof(PREAMBLE_PATTERN) / sizeof(PREAMBLE_PATTERN[0]);

static const uint8_t MESSAGE_BYTE = 'a';                                            // payload byte to send each frame
static const uint32_t IDLE_MS = ((PREAMBLE_MS * PREAMBLE_LEN) + (DATA_MS * 8)) / 4; // idle/silence before & after each frame
static const uint32_t PREAMBLE_GAP_MS = 1000;                                       // gap between preamble and message (must match receiver's PREAMBLE_GAP_MS)

// ---- Direct-register pin control (replaces pinMode()/digitalWrite() on the
// hot path). CHIRP_PIN == D11 == PB3 on the ATmega328 Nano -- these two
// helpers hardcode that mapping instead of looking it up generically. ----

static inline void chirpPinSetup()
{
  DDRB |= (1 << DDB3);  // D11 as output
  PORTB &= ~(1 << PB3); // idle LOW
}

static inline void chirpPinIdleLow()
{
  PORTB &= ~(1 << PB3);
}

// ---- Timer1-ISR-driven sweep state ----
// Timer1 fires a compare-match interrupt once per sweep step; the ISR
// itself computes and writes the next OCR2A value, so step timing is
// governed by hardware compare matches rather than a polled busy-wait.
static volatile uint16_t isrStepIndex;
static volatile bool isrSweepDone;
static bool isrUp;
static uint16_t isrSpan; // OCR_LOW - OCR_HIGH, precomputed once per chirp

ISR(TIMER1_COMPA_vect)
{
  if (isrStepIndex < STEPS)
  {
    int32_t val = isrUp ? (int32_t)OCR_LOW - ((int32_t)isrSpan * isrStepIndex) / STEPS
                        : (int32_t)OCR_HIGH + ((int32_t)isrSpan * isrStepIndex) / STEPS;
    OCR2A = (uint8_t)val;
    isrStepIndex++;
  }
  else
  {
    TIMSK1 &= ~(1 << OCIE1A); // done -- stop firing until the next chirp arms it again
    isrSweepDone = true;
  }
}

// Arms Timer1 (CTC, prescaler /64) to fire every stepUs microseconds.
// Prescaler 64 comfortably covers both DATA_MS and PREAMBLE_MS step
// intervals (a few ms each) without OCR1A overflowing 16 bits.
static void startStepTimer(uint32_t stepUs)
{
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  uint32_t ocr = ((F_CPU / 64UL) * stepUs) / 1000000UL;
  if (ocr > 0)
    ocr -= 1;
  OCR1A = (uint16_t)ocr;
  TCCR1B |= (1 << WGM12);              // CTC, TOP = OCR1A
  TIMSK1 |= (1 << OCIE1A);             // enable compare-match interrupt
  TCCR1B |= (1 << CS11) | (1 << CS10); // start timer, prescaler /64
}

static void stopStepTimer()
{
  TCCR1B = 0;
  TIMSK1 &= ~(1 << OCIE1A);
}

// Hand-written cycle-counted busy-wait for the last stretch of the
// end-of-chirp pad. A micros()-polled wait can only stop sometime after
// the deadline, by however long its own loop body + micros() call takes
// (micros() itself costs a handful of cycles to read the timer with
// interrupts briefly disabled). For a coarse wait that overshoot doesn't
// matter, so we still use micros()-polling down to ~50us left; but for
// that last short stretch we switch to this tighter, deterministic
// cycle-counted loop instead. Each iteration is sbiw (2 cycles) + brne
// (2 cycles taken, 1 not taken) = 4 cycles, so pass cycles/4.
static inline void delayCyclesAsm(uint16_t cycles)
{
  uint16_t count = cycles / 4;
  asm volatile(
      "1: sbiw %0, 1"
      "\n\t"
      "brne 1b"
      "\n\t"
      : "=w"(count)
      : "0"(count));
}

// ---- Timer2 CTC toggle setup ----

void timer2Init()
{
  chirpPinSetup();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  // CTC mode, TOP = OCR2A  (WGM22:0 = 0b010)
  TCCR2A |= (1 << WGM21);
}

void startToggle(uint8_t initialOcr)
{
  OCR2A = initialOcr;
  TCNT2 = 0;
  TCCR2A |= (1 << COM2A0); // toggle OC2A on compare match
  TCCR2B |= (1 << CS20);   // start timer, no prescaling (needed for the high-frequency band)
}

void stopToggle()
{
  TCCR2B = 0;               // stop the clock
  TCCR2A &= ~(1 << COM2A0); // disconnect OC2A, pin returns to normal I/O
  chirpPinIdleLow();        // force idle low between/after chirps
}

// ---- High-level chirp / frame primitives ----

void sendChirp(bool up, uint32_t durationMs)
{
  Serial.print(F("  chirp ("));
  Serial.print(durationMs);
  Serial.print(F("ms): "));
  Serial.println(up ? F("UP   (bit=1)") : F("DOWN (bit=0)"));

  uint32_t stepUs = (durationMs * 1000UL) / STEPS;

  uint8_t startOcr = up ? OCR_LOW : OCR_HIGH; // OCR_LOW=lowest freq, OCR_HIGH=highest freq
  isrUp = up;
  isrSpan = OCR_LOW - OCR_HIGH;
  isrStepIndex = 1; // step 0 is the initial OCR2A write below, ISR starts from step 1
  isrSweepDone = false;

  uint32_t t0 = micros();
  startToggle(startOcr);
  startStepTimer(stepUs);

  // The ISR drives every OCR2A update from here; the main "loop" below is
  // just waiting for it to finish, not pacing it.
  while (!isrSweepDone)
  {
  }
  stopStepTimer();

  // Pad out to exactly durationMs so the receiver's fixed windows line up.
  // Coarse micros()-polled wait down to ~50us left, then a tight
  // cycle-counted asm delay for the remainder (see delayCyclesAsm above).
  uint32_t frameEnd = t0 + durationMs * 1000UL;
  uint32_t coarseTarget = frameEnd - 50;
  while ((int32_t)(micros() - coarseTarget) < 0)
  {
  }
  int32_t remainingUs = (int32_t)(frameEnd - micros());
  if (remainingUs > 0)
    delayCyclesAsm((uint16_t)remainingUs * (F_CPU / 1000000UL));

  stopToggle();
}

void idleSilence(uint32_t ms)
{
  Serial.print(F("idle "));
  Serial.print(ms);
  Serial.println(F(" ms"));
  delay(ms);
}

// A "symbol" is REPEATS identical chirps sent back-to-back. The receiver
// integrates across all of them before deciding the bit -- this is the
// spreading/processing-gain step.
void sendSymbol(bool up, uint32_t chirpMs)
{
  Serial.print(F("symbol: "));
  Serial.print(REPEATS);
  Serial.print(F("x "));
  Serial.print(chirpMs);
  Serial.print(F("ms chirps, "));
  Serial.println(up ? F("UP   (bit=1)") : F("DOWN (bit=0)"));
  for (uint8_t r = 0; r < REPEATS; r++)
    sendChirp(up, chirpMs);
}

void sendPreamble()
{
  Serial.print(F("--- preamble ("));
  Serial.print(PREAMBLE_LEN);
  Serial.println(F(" symbols) ---"));
  for (uint8_t i = 0; i < PREAMBLE_LEN; i++)
    sendSymbol(PREAMBLE_PATTERN[i], PREAMBLE_MS);
}

// Even parity: parity bit set so that the total number of 1-bits
// (8 data bits + parity bit) is even. Lets the receiver catch single-bit errors.
uint8_t evenParity(uint8_t b)
{
  uint8_t p = 0;
  for (uint8_t i = 0; i < 8; i++)
    p ^= (b >> i) & 0x01;
  return p;
}

void sendByte(uint8_t b)
{
  uint8_t parity = evenParity(b);
  Serial.print(F("--- message byte 0x"));
  Serial.print(b, HEX);
  Serial.print(F(" ('"));
  Serial.write(b);
  Serial.print(F("') parity="));
  Serial.println(parity);

  for (int8_t i = 7; i >= 0; i--)
  {
    bool bit = (b >> i) & 0x01;
    sendSymbol(bit, DATA_MS);
  }
  sendSymbol(parity, DATA_MS); // 9th symbol: parity bit
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println(F("=== Chirp Sender ready ==="));
  timer2Init();
}

void loop()
{
  idleSilence(IDLE_MS); // sync gap: gives receiver a clean quiet reference
  sendPreamble();
  idleSilence(PREAMBLE_GAP_MS); // gap between preamble and message, per spec
  sendByte(MESSAGE_BYTE);
  idleSilence(IDLE_MS); // gap before next frame
  Serial.println(F("=== frame complete, repeating ==="));
}