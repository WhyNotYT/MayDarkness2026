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
 *     10-bit (0-1023). The FIFO path below keeps the full native resolution
 *     (no right-shift), which is a free sensitivity win for the
 *     mean-abs-delta activity metric (finer steps = more granularity to
 *     detect weak signal-driven jitter above noise).
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
 * edge averages.
 *
 * SYMBOL REDUNDANCY (matches sender):
 *   Each logical bit is transmitted as a "symbol" of SYMBOL_REPEATS
 *   back-to-back identical chirps (all UP or all DOWN), not a single
 *   chirp. We decode each of the sub-chirps independently and
 *   majority-vote the result for the symbol -- this makes bit decisions
 *   robust to a single bad/noisy sub-chirp.
 *
 * HARDWARE NOTE:
 *   A floating ADC pin picks up ambient noise (mains hum, nearby wires)
 *   that can swing wildly and falsely look like a signal. Add a ~1M ohm
 *   resistor from GP26 to GND to give it a stable baseline when nothing is
 *   coupling in.
 *
 * SYNC STRATEGY -- continuous up-chirp scan + preamble-tail lock:
 *   The old approach waited for a quiet baseline, then waited for an
 *   amplitude-threshold "onset" before starting a clock-aligned preamble
 *   decode from symbol 0. That has two weaknesses: it needs a calibrated
 *   noise floor, and it needs to catch the *exact* start of the frame or
 *   every subsequent aligned window is off.
 *
 *   Instead, this version never stops sampling. It continuously classifies
 *   back-to-back free-running chirp-length windows (no waiting for a clock
 *   edge, no quiet-baseline requirement) looking for the shape of an UP
 *   chirp. EXPECTED_PREAMBLE is {UP,UP,UP,UP,DOWN,DOWN}, so an UP-shaped hit
 *   can land anywhere within the first four preamble symbols -- we don't
 *   care which one, or whether we caught it mid-symbol; catching just one
 *   sub-chirp anywhere in that run is enough to kick off sync, so it's fine
 *   to miss a few chirps at the very start of the frame.
 *
 *   Once triggered, we keep classifying consecutive free-running windows
 *   and count a run of confident DOWN classifications. SYMBOL_REPEATS*2
 *   consecutive DOWN sub-chirps only happens in one place in the pattern:
 *   the preamble's last two symbols (the trailing false,false). That run
 *   length is therefore an unambiguous tail marker. Because chirp length is
 *   fixed and known (PREAMBLE_MS), the instant we confirm that tail we can
 *   compute the frame's t0 by walking back PREAMBLE_LEN symbols from *now*
 *   -- without ever having needed to catch the frame's true onset. From
 *   there, everything downstream (preamble re-verification, data bits,
 *   parity) uses the original clock-aligned, millis()-synchronized decode
 *   path, since that's what stays correctly aligned to the sender's bin
 *   windows.
 *
 * FAST ADC (hardware FIFO, free-running, 500 kHz):
 *   analogRead() re-does the whole ADC init/select/start/wait dance on
 *   every single call, which is slow (tens of us of overhead on top of the
 *   actual conversion) and jittery, since that overhead isn't constant --
 *   bad for both the continuous scanner (which needs to keep up in real
 *   time) and the bin-timed decode (which needs evenly-spaced samples).
 *   Instead the ADC free-runs into its hardware FIFO at a fixed rate with
 *   no software involved between conversions; we just pop samples out of
 *   the FIFO (fastAdcRead()) wherever the old code called analogRead().
 *
 * CALIBRATION:
 *   None needed anymore -- there's no amplitude threshold to calibrate.
 *   Detection is purely shape-based (chirp-vs-ramp correlation), which is
 *   self-normalizing: MIN_CORRELATION controls confidence the same way it
 *   always has for bit decisions, and now also gates the scan/sync steps.
 *   If your log shows |r| values only just above 0 for real chirps, lower
 *   it; if ambiguous calls are frequently wrong, raise it.
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
 *
 * PERFORMANCE NOTES (RP2040/Pico W side -- see feature flags below):
 *   - ENABLE_OVERCLOCK: raises sys_clk (default 250MHz vs. stock 125MHz)
 *     via vreg_set_voltage()+set_sys_clock_khz(). The RP2040 ADC's own
 *     clock is derived internally at a fixed 48MHz regardless of sys_clk,
 *     so overclocking does NOT change ADC sample timing/validity -- the
 *     benefit here is just more CPU headroom for core1's logging and for
 *     the correlation math (now running continuously, not just during
 *     decode), not faster sampling. 250MHz is a commonly reported safe
 *     headroom for RP2040 but isn't guaranteed per-chip; drop to 200000 in
 *     OVERCLOCK_KHZ if you see instability (garbled Serial output, hangs).
 *   - ENABLE_SMPS_FORCE_PWM: forces the onboard buck-boost regulator into
 *     PWM mode instead of its default light-load PFM mode, which reduces
 *     ripple on the ADC's supply rail (real effect, documented in the
 *     Pico datasheet). On the plain Pico this is a direct GPIO23 write;
 *     on the *W*, that same control line (WL_GPIO1) got repurposed for
 *     the wireless module wiring and is only reachable through the cyw43
 *     driver's GPIO-expander API -- so enabling this pulls in the WiFi
 *     driver stack (extra flash/RAM and ~100ms init time) even though we
 *     never connect to a network. Off by default for that reason; flip it
 *     on if you've actually observed ADC noise correlated with SMPS
 *     switching (a periodic ripple pattern in idle activity readings) and
 *     want to trade some efficiency to test whether it's the cause.
 *   - ENABLE_DUAL_CORE_LOGGING: core1 drains a small queue of pre-formatted
 *     log lines via loop1() and does the actual Serial.println(), so
 *     core0 (running the timing-critical sampling loops -- now including
 *     the continuous scanner, which never idles) never blocks on USB CDC
 *     I/O. Only the richest/most frequent print (the per-bin-activity line
 *     in classifyChirp()) was migrated to the queue; the rest are low
 *     frequency (once per symbol/byte/frame) and were left as direct
 *     Serial calls since there's nothing to gain there.
 */

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"

// ---- Performance feature flags -- see header comment above for what each
// one does and its tradeoffs. All independent; toggle any subset. ----
#define ENABLE_OVERCLOCK 1
#define OVERCLOCK_KHZ 250000
#define ENABLE_SMPS_FORCE_PWM 0
#define ENABLE_DUAL_CORE_LOGGING 1

#if ENABLE_SMPS_FORCE_PWM
#include "pico/cyw43_arch.h"
#endif

static const uint8_t ADC_PIN = A0;           // GP26 on Pico W
static const uint32_t PREAMBLE_MS = 500;     // duration of each sub-chirp in a preamble symbol
static const uint32_t DATA_MS = 750;         // duration of each sub-chirp in a data symbol (must match sender's DATA_MS)
static const uint8_t SYMBOL_REPEATS = 3;     // sub-chirps per symbol (must match sender)
static const uint32_t PREAMBLE_GAP_MS = 100; // idle gap between preamble and message (matches sender)
static const uint8_t BINS = 10;
static const float MIN_CORRELATION = 0.01f; // TUNE THIS -- Pearson r magnitude below this is flagged [ambiguous] / not confident enough to count during scan+sync

// ---- Protocol config (EVERYTHING here must match the sender's copy) ----

// Must be byte-for-byte identical to the sender's PREAMBLE_PATTERN[].
// Length is derived automatically -- just edit the list.
static const bool EXPECTED_PREAMBLE[] = {true, true, true, true, false, false};
static const uint8_t PREAMBLE_LEN = sizeof(EXPECTED_PREAMBLE) / sizeof(EXPECTED_PREAMBLE[0]);

// ---- Fast ADC (hardware FIFO, free-running @ 500kHz) ----
// See header notes: replaces analogRead() everywhere below with a FIFO pop
// from a free-running conversion stream, for both speed and even sample
// spacing -- important now that the scanner runs continuously, not just
// during a decode.
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
// everywhere below, but backed by the free-running FIFO above instead of a
// fresh conversion each call.
inline uint16_t fastAdcRead()
{
  while (adc_fifo_is_empty())
  { /* wait for next free-running conversion */
  }
  return adc_fifo_get();
}

// ---- Dual-core logging: core1 drains this queue and does the actual
// Serial.println() work, so core0's timing-critical sampling loops never
// touch USB CDC I/O. ----
#define LOG_LINE_MAX 128

#if ENABLE_DUAL_CORE_LOGGING
#define LOG_QUEUE_LEN 8
static volatile char logQueue[LOG_QUEUE_LEN][LOG_LINE_MAX];
static volatile uint8_t logHead = 0; // producer index (core0)
static volatile uint8_t logTail = 0; // consumer index (core1)

void qlog(const char *fmt, ...)
{
  uint8_t next = (logHead + 1) % LOG_QUEUE_LEN;
  if (next == logTail)
    return; // queue full -- drop the line rather than ever block core0
  va_list args;
  va_start(args, fmt);
  vsnprintf((char *)logQueue[logHead], LOG_LINE_MAX, fmt, args);
  va_end(args);
  logHead = next;
}

void setup1()
{
  // nothing to initialize -- core1 only drains the log queue
}

void loop1()
{
  if (logTail != logHead)
  {
    Serial.println((const char *)logQueue[logTail]);
    logTail = (logTail + 1) % LOG_QUEUE_LEN;
  }
}
#else
// Fallback so call sites don't need #ifdefs: just print directly on core0.
void qlog(const char *fmt, ...)
{
  char buf[LOG_LINE_MAX];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.println(buf);
}
#endif

// ---- Chirp classification (matched-filter / correlation decoding) ----
//
// Both variants below slice a chirp-length window into BINS bins, compute
// mean-abs-sample-to-sample-delta "activity" per bin, then correlate the
// resulting profile against an ideal rising-ramp template (Pearson r).
// r > 0 => UP chirp (bit 1); r < 0 => DOWN chirp (bit 0); |r| near 0 means
// the profile didn't look like either and the call is a coin flip (flagged
// [ambiguous] when |r| < MIN_CORRELATION).
//
//   - classifyChirpFreeRunning(): used by the continuous scanner and the
//     preamble-tail sync step. Its window starts "now" (whenever it's
//     called) and runs back-to-back with the previous call -- no waiting
//     for a clock edge, because at that point we don't have one to wait
//     for yet.
//   - sampleChirpBit(): used once we're locked (t0 known) to decode actual
//     preamble/data symbols. Its window is aligned to an absolute
//     millis() timestamp handed down from t0, matching the sender's bin
//     boundaries exactly.

// Shared bin-profile -> Pearson r helper.
float correlateRisingRamp(const float *binActivity)
{
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
  return (denom > 1e-6f) ? (covariance / denom) : 0.0f;
}

// Classifies one free-running chirp-length window starting *now* (no wait
// for alignment -- callers chain these back-to-back for continuous
// scanning). Returns bit (r>0 => UP/1) and reports the correlation via
// rOut so callers can gate on confidence (|r| >= MIN_CORRELATION).
bool classifyChirpFreeRunning(uint32_t durationMs, float &rOut)
{
  float binActivity[BINS];
  uint32_t binMs = durationMs / BINS;

  for (uint8_t b = 0; b < BINS; b++)
  {
    uint32_t binEnd = millis() + binMs;
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

  rOut = correlateRisingRamp(binActivity);
  return rOut > 0;
}

// Samples one *single* chirp window aligned to windowStart (millis()), of
// the given duration, and returns true for UP (bit=1), false for DOWN
// (bit=0). Used post-lock, once t0 is known, so windows land exactly on
// the sender's bin boundaries.
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

  float r = correlateRisingRamp(binActivity);

  char logbuf[LOG_LINE_MAX];
  int off = snprintf(logbuf, sizeof(logbuf), "    activity/bin: ");
  for (uint8_t b = 0; b < BINS && off < (int)sizeof(logbuf); b++)
    off += snprintf(logbuf + off, sizeof(logbuf) - off, "%.1f ", binActivity[b]);
  snprintf(logbuf + off, sizeof(logbuf) - off, "| r=%.3f%s", r,
           (fabs(r) < MIN_CORRELATION) ? " [ambiguous]" : "");
  qlog("%s", logbuf);

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

// ---- Sync: continuous up-chirp scan + preamble-tail lock ----
//
// See header comment for the full rationale. Summary:
//   1) scanForUpChirp(): free-runs chirp-length windows forever until one
//      classifies as a confident UP chirp. Could be any of the preamble's
//      first four (UP) symbols, or any sub-chirp within one -- doesn't
//      matter which, so it's fine to miss a few at the very start.
//   2) syncToPreambleTail(): from there, keeps free-running windows and
//      counts a run of confident DOWN classifications. SYMBOL_REPEATS*2
//      consecutive DOWN sub-chirps only ever happens at the preamble's
//      last two symbols (EXPECTED_PREAMBLE's trailing false,false), so
//      that run length unambiguously marks the tail. Chirp length is
//      fixed/known, so t0 = now - (PREAMBLE_LEN full symbols).

bool checkStreamCommand(); // fwd decl (debug hook, defined below)

// Continuously scans free-running chirp-length windows for an UP-chirp
// shape. Returns true the moment one window classifies as UP with
// sufficient confidence. Returns false only if a debug stream command
// interrupted the scan (so loop() can go handle it and come back).
bool scanForUpChirp()
{
  while (true)
  {
    if (checkStreamCommand())
      return false;

    float r;
    bool bit = classifyChirpFreeRunning(PREAMBLE_MS, r);
    if (bit && r >= MIN_CORRELATION)
    {
      qlog("[scan] up chirp candidate detected (r=%.3f)", r);
      return true;
    }
  }
}

// After a candidate up chirp, keeps classifying consecutive free-running
// chirp windows and counts a run of confident DOWN classifications. Once
// the run reaches SYMBOL_REPEATS*2 (the preamble's last two DOWN symbols),
// computes t0 by walking back PREAMBLE_LEN symbols from now and returns
// true. Gives up (returns false) if the tail isn't found within
// PREAMBLE_MS * PREAMBLE_LEN of real time (the longest the whole preamble
// could legitimately take), so a false-positive up-chirp trigger (noise)
// or a run that never completes doesn't wedge the receiver forever -- it
// just goes back to scanning.
bool syncToPreambleTail(uint32_t &t0Out)
{
  const uint8_t DOWN_TAIL_SUBCHIRPS = SYMBOL_REPEATS * 2; // last 2 DOWN symbols, back-to-back
  const uint32_t TIMEOUT_MS = PREAMBLE_MS * PREAMBLE_LEN;

  uint32_t deadline = millis() + TIMEOUT_MS;
  uint8_t downRun = 0;
  while ((int32_t)(millis() - deadline) < 0)
  {
    if (checkStreamCommand())
      return false;

    float r;
    bool bit = classifyChirpFreeRunning(PREAMBLE_MS, r);
    bool confidentDown = (!bit) && (fabs(r) >= MIN_CORRELATION);
    bool confidentUp = bit && (r >= MIN_CORRELATION);

    qlog("[tail] classified %s r=%.3f%s -- downRun=%u/%u",
         bit ? "UP" : "DOWN", r,
         (confidentDown || confidentUp) ? "" : " [ambiguous]",
         (unsigned)(confidentDown ? downRun + 1 : downRun), (unsigned)DOWN_TAIL_SUBCHIRPS);

    if (confidentDown)
    {
      downRun++;
      if (downRun >= DOWN_TAIL_SUBCHIRPS)
      {
        uint32_t preambleTotalMs = (uint32_t)PREAMBLE_LEN * PREAMBLE_MS * SYMBOL_REPEATS;
        t0Out = millis() - preambleTotalMs;
        qlog("[sync] preamble tail found (%u down sub-chirps) -> t0=%lu",
             (unsigned)DOWN_TAIL_SUBCHIRPS, (unsigned long)t0Out);
        return true;
      }
    }
    else if (confidentUp)
    {
      // Still inside (or re-entered) the UP run -- not at the tail yet.
      downRun = 0;
    }
    // Ambiguous windows neither extend nor reset the run: a single noisy
    // sub-chirp mid-tail shouldn't be able to break an otherwise-good lock.
  }
  qlog("[!] preamble tail not found within %lu ms -- abandoning sync attempt", (unsigned long)TIMEOUT_MS);
  return false;
}

// ---- Frame-level logic ----

void setup()
{
#if ENABLE_OVERCLOCK
  // Raise core voltage slightly before raising the clock -- standard
  // pattern for RP2040 overclocking headroom. Do this before Serial.begin()
  // since it's unrelated to USB (USB uses its own fixed-rate PLL).
  vreg_set_voltage(VREG_VOLTAGE_1_15);
  delay(10);
  set_sys_clock_khz(OVERCLOCK_KHZ, true);
#endif

  Serial.begin(115200);
  // Pico W's Serial is native USB CDC. Give the host a few seconds to
  // enumerate/open the port so early prints aren't lost, but don't hang
  // forever if nothing's connected (e.g. running off USB power only).
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000)
  { /* wait for USB CDC */
  }
  delay(200);

#if ENABLE_SMPS_FORCE_PWM
  cyw43_arch_init();
  cyw43_arch_gpio_put(1, true); // WL_GPIO1 = SMPS PS pin; HIGH forces PWM mode (lower ADC-supply ripple, worse efficiency)
  Serial.println(F("[pwr] SMPS forced to PWM mode (WL_GPIO1 high) for lower ADC ripple"));
#endif

  setupFastAdc(); // free-running FIFO ADC @ 500kHz, replaces analogRead() everywhere below

  Serial.println(F("=== Chirp Receiver ready (Pico W) ==="));
  Serial.println(F("[scan] continuously scanning for up chirps -- no quiet baseline needed..."));
}

// Non-blocking check for the 's' stream-dump command. Kept as a hook point
// so a debug/visualizer build can plug in without touching the sync logic;
// this build has nothing to stream, so it's a no-op that just drains any
// pending input.
bool checkStreamCommand()
{
  if (Serial.available())
  {
    Serial.read();
  }
  return false;
}

void loop()
{
  // --- Phase 1: scan continuously for an up chirp anywhere in the preamble's UP run ---
  if (!scanForUpChirp())
    return;

  Serial.println(F("[sync] up chirp seen -- syncing to preamble tail (last 2 down chirps)..."));

  // --- Phase 2: lock onto the preamble's trailing down-chirp run, derive t0 ---
  uint32_t t0;
  if (!syncToPreambleTail(t0))
  {
    Serial.println(F("[sync] lost lock before finding the tail -- resuming scan"));
    return;
  }
  Serial.println(F("[sync] locked -- frame t0 established"));

  uint32_t preambleSymbolMs = PREAMBLE_MS * SYMBOL_REPEATS;
  uint32_t dataSymbolMs = DATA_MS * SYMBOL_REPEATS;

  // NOTE: there's no separate "verify preamble" pass here. By the time
  // syncToPreambleTail() confirms the tail, the entire preamble is already
  // in the past -- sampleChirpBit()'s alignment wait can't rewind the ADC
  // to re-sample chirps that already happened (binStart/binEnd would
  // already be behind millis(), so every bin would read zero samples,
  // which is exactly the all-zero/[ambiguous] readout this replaces).
  // The live UP,UP,UP,UP,DOWN,DOWN classification we just did during
  // scanForUpChirp()/syncToPreambleTail() *is* the preamble verification;
  // we go straight to the message, which -- unlike the preamble -- is
  // correctly in the future relative to t0.
  Serial.println(F("[ok] preamble confirmed via live scan/tail classification"));

  // --- gap, then 8 data bits + 1 parity bit, each a symbol, MSB first ---
  uint32_t messageStart = t0 + (uint32_t)PREAMBLE_LEN * preambleSymbolMs + PREAMBLE_GAP_MS;
  uint8_t byteVal = 0;

  Serial.println(F("--- decoding message ---"));
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

  Serial.println(F("[scan] resuming continuous up-chirp scan..."));
}