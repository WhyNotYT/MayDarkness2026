/*
 * CHIRP RECEIVER -- ESP32 WROOM-32 port (I2S ADC DMA + dual-core split)
 * ----------------------------------------------------------------------
 * Reads ADC1_CHANNEL_0 (GPIO36, silkscreened "VP" on most WROOM-32 dev
 * boards) via the ESP32's I2S peripheral running in "built-in ADC" DMA
 * mode, and decodes UP/DOWN chirps into bits using the same
 * capacitive-coupling amplitude-trend technique as the original AVR/Pico
 * receivers. Decode logic is unchanged; everything platform-specific
 * (ADC acquisition path, core assignment, ring buffer) is new.
 *
 * WHY I2S FOR THE ADC:
 *   analogRead()/adc1_get_raw() on the ESP32 do a full blocking conversion
 *   per call (~10-20us each with software overhead on top), which caps you
 *   at maybe ~50-100kHz *and* is jittery -- exactly the problem the Pico
 *   port solved with a free-running FIFO. The ESP32's I2S peripheral has a
 *   hardware "ADC built-in" mode where the I2S clock directly paces the
 *   on-chip ADC1 and DMA's the raw conversions into RAM with zero CPU
 *   involvement per sample. That's the ESP32 equivalent of the Pico's
 *   free-running FIFO, and it's what lets us push the sample clock far
 *   higher than any register-poking loop could.
 *
 * ON THE "2MHz" TARGET:
 *   i2s_config.sample_rate is a real register value we can set as high as
 *   the I2S clock tree allows -- 2,000,000 is achievable as a *clock*
 *   config on this silicon. Whether the on-chip SAR ADC actually settles
 *   to a valid 12-bit conversion at that pace is a separate question --
 *   Espressif's own datasheet only characterizes ADC1 accuracy up to
 *   ~2MSPS in single-shot friendly conditions, and in practice most
 *   projects run I2S-ADC DMA in the hundreds-of-kHz range before dropped/
 *   garbage samples show up. FAST_ADC_SAMPLE_RATE_HZ below is left at
 *   2,000,000 to hit the ask ("theoretically go to 2MHz") -- if you see
 *   DMA overruns (logged) or the decoded activity profile looks like pure
 *   noise, back this down (500000-1000000 is a safer starting point) and
 *   verify empirically, same caveat the Pico port left for analogRead()
 *   timing.
 *
 * WHY GPIO36 / ADC1:
 *   ADC2 shares hardware with the WiFi radio and is unusable (or flaky)
 *   any time WiFi is active, and the I2S-ADC DMA path only supports ADC1
 *   channels anyway. GPIO36 (ADC1_CH0, "VP") is a fixed, input-only pin
 *   with no pull resistor -- same "floating pin picks up ambient noise"
 *   caveat as the Pico's GP26 applies here, so keep the ~1M ohm pulldown
 *   to GND from the original hardware note.
 *
 * DUAL-CORE SPLIT (per your ask):
 *   The WROOM-32 is a dual-core Xtensa LX6. Core 0 is dedicated
 *   *exclusively* to pulling finished DMA buffers out of the I2S driver
 *   and copying/masking them into a lock-free single-producer/
 *   single-consumer ring buffer, using a small hand-written Xtensa inline
 *   assembly loop for that copy (see asmCopySamplesToRing() below) so the
 *   only C/C++ overhead per buffer is the i2s_read() call itself -- no
 *   compiler-generated bounds-checking, no function-call-per-sample, no
 *   dead time where core 0 could stall the DMA consumer and let the
 *   hardware ring buffer overflow. Core 1 runs everything else: onset
 *   detection, chirp bit correlation, framing/protocol state machine, and
 *   all Serial I/O -- i.e. it's a straight port of the original decode
 *   loop, just reading from the ring buffer instead of calling
 *   fastAdcRead() directly.
 *
 *   NOTE: the true "sampling" (the actual ADC conversions) is done by
 *   dedicated I2S/ADC hardware via DMA, not by software on core 0 at all
 *   -- that's the only way to approach MHz-class rates on this chip. What
 *   runs in assembly on core 0 is the *hot extraction loop* that drains
 *   finished DMA buffers into our own ring buffer as fast as possible, so
 *   core 0 never becomes the bottleneck between the DMA hardware and
 *   core 1's decoder.
 *
 * SYMBOL REDUNDANCY, MATCHED-FILTER BIT DECODING, ONSET DETECTION,
 * CALIBRATION, and the PROTOCOL FRAMING are all unchanged from the Pico
 * port -- see the comments above sampleChirpBit()/sampleSymbolBit()/
 * calibrateNoiseFloor() below; only the acquisition path changed.
 *
 * HARDWARE NOTE:
 *   Wire the antenna stub / pickup wire to GPIO36 ("VP"), share GND with
 *   the sender, and add a ~1M ohm resistor from GPIO36 to GND so the pin
 *   has a stable baseline when nothing is coupling in.
 *
 * Protocol expected (must mirror the sender exactly; unchanged from the
 * Pico port):
 *   [idle]
 *   -> [preamble: PREAMBLE_LEN symbols @ PREAMBLE_MS/sub-chirp x SYMBOL_REPEATS
 *                 repeats, pattern EXPECTED_PREAMBLE[]]
 *   -> [PREAMBLE_GAP_MS idle]
 *   -> [message: 8 data bits + 1 parity bit, each a symbol @ DATA_MS/sub-chirp
 *                x SYMBOL_REPEATS repeats, MSB first]
 *   -> [idle] -> repeat
 */

#include <Arduino.h>
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_task_wdt.h"

// ---- Debug/streaming support ----
// Typing 's' + Enter on the serial console dumps STREAM_SAMPLE_COUNT raw
// ADC samples as binary uint16 (little-endian) instead of running the
// decoder, so an external tool (e.g. the companion Python visualizer) can
// plot the waveform/spectrum/correlation live. Framed as: 4 sync bytes
// 0xAA 0x55 0xAA 0x55, then a uint32 sample count, then the raw uint16
// samples, then 0xFF x4 as an end marker.
static const uint32_t STREAM_SAMPLE_COUNT = 1000; // ~ a couple of chirp windows worth

// ---- ADC / I2S config ----
static const adc1_channel_t ADC_CHANNEL = ADC1_CHANNEL_0; // GPIO36 / "VP"
static const uint8_t ADC_RESOLUTION_BITS = 12;            // native ESP32 ADC1 resolution
static const uint32_t FAST_ADC_SAMPLE_RATE_HZ = 500000;   // see header note: "theoretical" 2MHz target
static const i2s_port_t I2S_ADC_PORT = I2S_NUM_0;

// I2S DMA buffering. Small-ish buffers (fewer samples per i2s_read call)
// keep latency down and give core 0's asm drain loop more, smaller bites
// rather than fewer, bigger ones -- helps keep the ring buffer topped up
// evenly instead of in bursts.
static const int I2S_DMA_BUF_COUNT = 8;
static const int I2S_DMA_BUF_LEN = 256; // samples per DMA buffer

// ---- Protocol / decode config (unchanged from the Pico port) ----
static const uint32_t PREAMBLE_MS = 2000;     // duration of each sub-chirp in a preamble symbol
static const uint32_t DATA_MS = 1000;         // duration of each sub-chirp in a data symbol (must match sender's DATA_MS)
static const uint8_t SYMBOL_REPEATS = 1;      // sub-chirps per symbol (must match sender)
static const uint32_t PREAMBLE_GAP_MS = 1500; // idle gap between preamble and message (matches sender)
static const uint8_t BINS = 10;
static const float MIN_CORRELATION = 0.2f;        // TUNE THIS -- Pearson r magnitude below this is flagged [ambiguous]
static const float MIN_ACTIVITY_THRESHOLD = 1.6f; // floor for auto-calibrated onset threshold
static const uint8_t ONSET_DEBOUNCE = 8;          // consecutive onset scans required
static const uint16_t ONSET_SAMPLES = 500;        // averaging window for onset activity metric

// Must be byte-for-byte identical to the sender's PREAMBLE_PATTERN[].
static const bool EXPECTED_PREAMBLE[] = {true, true, true, true, false, false};
static const uint8_t PREAMBLE_LEN = sizeof(EXPECTED_PREAMBLE) / sizeof(EXPECTED_PREAMBLE[0]);

float adcActiveThreshold = 1.0f; // overwritten by calibrateNoiseFloor() at startup

// =====================================================================
// ---- Lock-free single-producer (core 0) / single-consumer (core 1)
//      ring buffer of raw 12-bit ADC samples ----
// =====================================================================
// Power-of-two size so wraparound is a cheap AND mask, both in C and in
// the inline asm drain loop below. 8192 samples gives a decent cushion
// (core 1's chirp bin loops can occasionally fall behind for a few ms
// without the ring overflowing) without eating too much RAM.
static const uint32_t RING_SIZE = 8192;
static const uint32_t RING_MASK = RING_SIZE - 1;
static uint16_t ringBuf[RING_SIZE];

// Plain volatile 32-bit indices are sufficient for lock-free SPSC on the
// Xtensa LX6 -- aligned 32-bit loads/stores are atomic on this core, and
// there's exactly one writer (core 0) and one reader (core 1) for each
// index, so no CAS/mutex is needed. head = next slot the producer will
// write, tail = next slot the consumer will read.
static volatile uint32_t ringHead = 0;
static volatile uint32_t ringTail = 0;
static volatile uint32_t ringOverruns = 0; // diagnostic counter, printed on demand

static inline uint32_t ringAvailable()
{
  return (ringHead - ringTail) & RING_MASK;
}

// =====================================================================
// ---- Core 0: I2S ADC DMA acquisition + inline-assembly drain loop ----
// =====================================================================

void setupI2sAdc()
{
  i2s_config_t i2sConfig = {};
  i2sConfig.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
  i2sConfig.sample_rate = FAST_ADC_SAMPLE_RATE_HZ;
  i2sConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;

  i2sConfig.channel_format = I2S_CHANNEL_FMT_ALL_LEFT; // was ONLY_RIGHT
  i2sConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2sConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2sConfig.dma_buf_count = I2S_DMA_BUF_COUNT;
  i2sConfig.dma_buf_len = I2S_DMA_BUF_LEN;
  i2sConfig.use_apll = false;

  i2s_driver_install(I2S_ADC_PORT, &i2sConfig, 0, nullptr);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  adc1_config_width(ADC_WIDTH_BIT_12);
  // 11dB attenuation = full ~0-3.3V input range, matching the wide swing
  // a capacitive-coupled pickup wire can produce.
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
  i2s_adc_enable(I2S_ADC_PORT);
}

// Hand-written Xtensa LX6 inline assembly loop: drains `count` raw 16-bit
// I2S/ADC words from `src` into the ring buffer starting at `startIdx`,
// masking each word down to the 12-bit ADC value as it goes, and returns
// the new head index (already wrapped via RING_MASK). This is the "hot
// path" per the design note above -- everything here is scratch
// registers a8-a10 plus the three loop variables, one load + mask + store
// + three increments + a branch per sample, no C-level function call or
// bounds check per element.
//
// NOTE ON REGISTER USE: a8-a10 are ordinary scratch registers under the
// GCC Xtensa windowed-ABI inline-asm convention used by esp-idf/arduino-
// esp32's toolchain; the compiler is told (via the clobber list) that
// they're destroyed, so it won't rely on their prior contents across this
// block. If you retarget this to a different toolchain/ABI, re-verify
// that assumption before trusting it.
static inline uint32_t IRAM_ATTR asmCopySamplesToRing(const uint16_t *src, uint32_t startIdx, uint32_t count)
{
  uint16_t *ring = ringBuf;
  uint32_t idx = startIdx;
  uint32_t mask = RING_MASK;

  if (count == 0)
    return idx;

  __asm__ volatile(
      "1:                              \n\t"
      "l16ui   a8, %[src], 0           \n\t" // a8 = *src  (raw 16-bit I2S/ADC word)
      "movi    a9, 0x0FFF              \n\t" // a9 = 0x0FFF (12-bit ADC data mask)
      "and     a8, a8, a9              \n\t" // a8 &= 0x0FFF -> masked sample value
      "addx2   a10, %[idx], %[ring]    \n\t" // a10 = ring + idx*2  (halfword-indexed address)
      "s16i    a8, a10, 0              \n\t" // *(ring + idx) = a8
      "addi    %[src], %[src], 2       \n\t" // src++ (advance by one 16-bit word)
      "addi    %[idx], %[idx], 1       \n\t" // idx++
      "and     %[idx], %[idx], %[mask] \n\t" // idx &= RING_MASK  (wrap)
      "addi    %[count], %[count], -1  \n\t" // count--
      "bnez    %[count], 1b            \n\t" // loop while count != 0
      : [src] "+r"(src), [idx] "+r"(idx), [count] "+r"(count)
      : [ring] "r"(ring), [mask] "r"(mask)
      : "a8", "a9", "a10", "memory");

  return idx;
}

// Core-0 task: forever pulls a finished DMA buffer from the I2S driver
// and drains it into the ring buffer via the asm loop above. This task
// does nothing else -- no Serial, no decode logic, no delay() -- so it
// never gives the DMA hardware a chance to overrun the driver's internal
// buffers.
void samplingTask(void *pvParameters)
{
  static uint16_t dmaBuf[I2S_DMA_BUF_LEN];
  size_t bytesRead = 0;

  for (;;)
  {
    esp_err_t err = i2s_read(I2S_ADC_PORT, (void *)dmaBuf, sizeof(dmaBuf), &bytesRead, portMAX_DELAY);
    if (err != ESP_OK || bytesRead == 0)
      continue;

    uint32_t samplesRead = bytesRead / sizeof(uint16_t);

    // Guard against the ring buffer filling faster than core 1 drains it
    // (e.g. while core 1 is blocked printing a long Serial line). We
    // still perform the copy -- overwriting the oldest unread samples --
    // but count/log it so the decode side (or you, reading the log) knows
    // data was lost rather than silently producing a corrupted decode.
    uint32_t head = ringHead;
    if (ringAvailable() + samplesRead > RING_SIZE)
      ringOverruns++;

    head = asmCopySamplesToRing(dmaBuf, head, samplesRead);
    ringHead = head; // publish new head last, after all samples are written
  }
}

// =====================================================================
// ---- Core 1: everything else (ported near-verbatim from the Pico
//      version, just reading from the ring buffer instead of the RP2040
//      ADC FIFO) ----
// =====================================================================

// Blocking pop of the oldest unread sample from the ring buffer. Drop-in
// replacement for the Pico port's fastAdcRead(), backed by core 0's I2S
// DMA + asm drain pipeline instead of the RP2040's ADC FIFO.
inline uint16_t ringRead()
{
  while (ringTail == ringHead)
  {
    // Ring momentarily empty (core 0 hasn't drained a DMA buffer yet).
    // A tiny yield keeps this from starving core 0/other tasks if this
    // ever spins for a while; in normal operation this loop exits almost
    // immediately given the sample rate involved.
  }
  uint16_t v = ringBuf[ringTail];
  ringTail = (ringTail + 1) & RING_MASK;
  return v;
}

// Streams STREAM_SAMPLE_COUNT raw samples out over Serial as a simple
// binary frame for the companion Python visualizer. Pure debug utility.
void streamRawAdc()
{
  Serial.println(F("[stream] starting raw ADC dump..."));
  uint8_t sync[4] = {0xAA, 0x55, 0xAA, 0x55};
  Serial.write(sync, 4);
  uint32_t count = STREAM_SAMPLE_COUNT;
  Serial.write((uint8_t *)&count, 4);
  for (uint32_t i = 0; i < count; i++)
  {
    uint16_t v = ringRead();
    Serial.write((uint8_t *)&v, 2);
  }
  uint8_t endMarker[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  Serial.write(endMarker, 4);
  Serial.flush();
  Serial.println(F("[stream] done"));
}

// Mean absolute sample-to-sample delta over N reads -- same metric
// sampleChirpBit() uses per-bin for bit decisions.
float measureActivity(uint16_t samples)
{
  long sum = 0;
  int last = ringRead();
  for (uint16_t i = 0; i < samples; i++)
  {
    int v = ringRead();
    sum += abs(v - last);
    last = v;
  }
  return (float)sum / samples;
}

// Measures idle activity on the ADC pin over ~2s and sets
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

  adcActiveThreshold = max(mean + 5.0f * stddev, MIN_ACTIVITY_THRESHOLD);
  Serial.print(F("[cal] idle activity mean="));
  Serial.print(mean, 3);
  Serial.print(F(" stddev="));
  Serial.print(stddev, 3);
  Serial.print(F(" -> threshold set to "));
  Serial.println(adcActiveThreshold, 3);
}

bool detectSignalOnsetOnce()
{
  float activity = measureActivity(ONSET_SAMPLES);
  return activity > adcActiveThreshold;
}

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
// of the given duration, and returns true if amplitude activity
// correlates with a rising ramp (bit=1, UP chirp), false for a falling
// ramp (bit=0, DOWN chirp). See the original Pico port's comments on the
// Pearson-correlation matched filter -- unchanged here.
bool sampleChirpBit(uint32_t windowStart, uint32_t durationMs)
{
  float binActivity[BINS];
  uint32_t binMs = durationMs / BINS;

  for (uint8_t b = 0; b < BINS; b++)
  {
    uint32_t binStart = windowStart + (uint32_t)b * binMs;
    uint32_t binEnd = binStart + binMs;

    while ((int32_t)(millis() - binStart) < 0)
    {
    }

    long sum = 0;
    uint16_t count = 0;
    int last = ringRead();
    while ((int32_t)(millis() - binEnd) < 0)
    {
      int v = ringRead();
      sum += abs(v - last);
      last = v;
      count++;
    }
    binActivity[b] = count > 0 ? (float)sum / count : 0.0f;
  }

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
// chirpMs each, all carrying the same bit -- and returns the majority-
// voted bit.
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

// Non-blocking check for the 's' stream-dump command.
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

// ---- Frame-level logic (core 1 / Arduino setup+loop) ----

void setup()
{
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000)
  { /* give the host a moment to open the port */
  }
  delay(200);

  // Arduino-ESP32's setup()/loop() already run as a FreeRTOS task
  // ("loopTask") pinned to core 1 (APP_CPU) by the framework itself, so
  // everything below this point -- calibration, onset detection, chirp
  // decode, framing, Serial -- naturally executes on core 1 without any
  // extra pinning code. Core 0 (PRO_CPU) is reserved entirely for the
  // dedicated sampling task started here.
  setupI2sAdc();

  xTaskCreatePinnedToCore(
      samplingTask,             // task function
      "i2s_sampling",           // name
      4096,                     // stack size (words on some ports / bytes here -- generous for a tiny asm-heavy task)
      nullptr,                  // parameters
      configMAX_PRIORITIES - 1, // highest priority: never let anything preempt the DMA drain
      nullptr,                  // task handle (unused)
      0                         // CORE 0 -- dedicated sampling core
  );

  // Let the ring buffer fill a bit before we start pulling from it.
  delay(50);

  Serial.println(F("=== Chirp Receiver ready (ESP32 WROOM-32, I2S ADC DMA) ==="));
  Serial.println(F("[hint] type 's' + Enter any time to dump a raw ADC stream for the Python visualizer"));
  calibrateNoiseFloor();
  Serial.println(F("Waiting for quiet baseline, then listening for a frame..."));
}

void loop()
{
  if (checkStreamCommand())
  {
    return;
  }

  if (ringOverruns > 0)
  {
    Serial.print(F("[warn] ring buffer overruns since boot: "));
    Serial.println(ringOverruns);
  }

  Serial.println(F("[sync] waiting for quiet..."));
  while (detectSignalOnset())
  {
    checkStreamCommand();
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