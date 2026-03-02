#include <Arduino.h>
#include <BLEMidi.h>
#include <EEPROM.h>
#include <fft.h>
#include "driver/i2s.h"
#include "I2SMEMSSampler.h"
#include "mallet.h"

namespace
{
constexpr unsigned long kMidiLeadTimeMs = 1000;
constexpr unsigned long kBleTimestampRolloverMs = 8192;
constexpr unsigned long kCalibrationImpactTimeoutMs = 2500;
constexpr unsigned long kCalibrationMalletIdleTimeoutMs = 2500;
constexpr unsigned long kImpactDetectGuardMs = 90;
constexpr unsigned long kCalibrationHardMaxLagMs = 400;
constexpr unsigned long kCalibrationSoftLagSlackMs = 180;
constexpr unsigned long kCalibrationSoftMaxLagFloorMs = 300;
constexpr unsigned long kRingdownTimeoutMs = 1800;
constexpr unsigned long kRingdownStableMs = 80;
constexpr unsigned long kFftTimeoutMs = 1200;
constexpr unsigned long kMalletRetriggerGapMs = 90;
constexpr unsigned long kPitchCaptureOffsetMs = 600;
constexpr unsigned long kCalibrationInterStrikeDelayMs = 120;
constexpr unsigned long kCalibrationInterMalletDelayMs = 300;
constexpr unsigned long kCalibrationSoftRetryDelayMs = 60;
constexpr int kSampleRateHz = 16000;
constexpr int kAudioBufferSamples = 256;
constexpr int kFftLength = 4096;
constexpr int kCalibrationRepeats = 1;
constexpr float kMicEmaAlpha = 0.01f;
constexpr float kMicDcAlpha = 0.001f;
constexpr int kMicSlotTo24BitShiftBits = 8;
constexpr int kMicEffectiveBits = 24;
constexpr int kMicUnusedLowBits = 24 - kMicEffectiveBits;
constexpr int kMicGainShiftDefaultBits = 0;
constexpr int kMicGainShiftMaxBits = 8;
constexpr int32_t kImpactThresholdHardRise = 450;
constexpr int32_t kImpactThresholdSoftRise = 180;
constexpr int32_t kRingdownThresholdRise = 260;
constexpr int32_t kImpactThresholdHysteresis = 30;
constexpr int32_t kImpactNoiseRiseFloor = 120;
constexpr int32_t kImpactNoiseMultiplier = 3;
constexpr int32_t kHardMinRiseFloor = 12000;
constexpr int32_t kSoftMinRiseFloor = 15000;
constexpr int32_t kSoftMinRiseHardDivisor = 4;
constexpr uint16_t kSoftPowerInitial = 580;
constexpr uint16_t kSoftPowerStep = 120;
constexpr uint16_t kSoftPowerMax = 900;
constexpr int kSoftPowerAttempts = 4;
constexpr float kFftMinDetectHz = 90.0f;
constexpr float kFftMaxDetectHz = 2200.0f;
constexpr float kFftMinMagnitude = 20.0f;
constexpr bool kRunSelfCalibrationOnBootWhenMissing = true;
constexpr char kBleDeviceName[] = "MusicRobot";
constexpr uint32_t kCalibrationMagic = 0x52444232; // "RDB2"
constexpr uint16_t kCalibrationVersion = 2;

enum MicChannelSelect
{
  MIC_CHANNEL_LEFT = 0,
  MIC_CHANNEL_RIGHT = 1,
};

Mallet mallets[] = {
  Mallet(15, 52, 0, 900, 110, 0, 50, 512, 50),
  Mallet(4, 55, 1, 900, 110, 0, 50, 512, 50),
  Mallet(12, 57, 2, 900, 110, 0, 50, 512, 50),
  Mallet(32, 60, 3, 900, 110, 0, 50, 512, 50),
  Mallet(27, 62, 4, 900, 110, 0, 50, 512, 50),
  Mallet(26, 64, 5, 900, 110, 0, 50, 512, 50),
  Mallet(25, 65, 6, 900, 110, 0, 50, 512, 50),
  Mallet(2, 67, 7, 900, 110, 0, 50, 512, 50),
  Mallet(13, 69, 8, 900, 110, 0, 50, 512, 50),
  Mallet(33, 72, 9, 900, 110, 0, 50, 512, 50),
};

constexpr size_t kMalletCount = sizeof(mallets) / sizeof(mallets[0]);

struct CalibrationHeader
{
  uint32_t magic;
  uint16_t version;
  uint16_t malletCount;
  uint32_t writeCount;
};

struct CalibrationEntry
{
  int16_t midiPitch;
  uint16_t softPower;
  uint16_t hardPower;
  uint16_t softLagMs;
  uint16_t hardLagMs;
  uint32_t softImpact;
  uint32_t hardImpact;
  uint8_t valid;
  uint8_t reserved[3];
};

struct VelocityCalibrationModel
{
  uint16_t softPower;
  uint16_t hardPower;
  uint16_t softLagMs;
  uint16_t hardLagMs;
  uint32_t softImpact;
  uint32_t hardImpact;
  bool valid;
};

struct StrikeMeasurement
{
  unsigned long lagMs;
  int32_t peakRise;
  bool impactDetected;
  int32_t ambient;
  int32_t triggerThreshold;
  int32_t noiseP2P;
  int32_t minRiseRequired;
};

struct ScheduledStrike
{
  Mallet::StrikeProfile profile;
  unsigned long lagMs;
};

constexpr size_t kCalibrationHeaderAddress = 0;
constexpr size_t kCalibrationEntriesAddress = sizeof(CalibrationHeader);
constexpr size_t kCalibrationStorageBytes = sizeof(CalibrationHeader) + (sizeof(CalibrationEntry) * kMalletCount);
constexpr size_t kEepromBytes = 512;

static_assert(kCalibrationStorageBytes <= kEepromBytes, "Calibration EEPROM storage exceeds reserved bytes");

VelocityCalibrationModel gVelocityCalibration[kMalletCount];

I2SSampler *gSampler = nullptr;
TaskHandle_t gAudioWriterTaskHandle = nullptr;

volatile int32_t gFilteredMic = 0;
volatile int32_t gDynamicMicPeak = 1;
volatile float gNormalizedMic = 0.0f;
volatile int32_t gMicDc = 0;
volatile int32_t gMicLeftPeak = 0;
volatile int32_t gMicRightPeak = 0;
volatile int gMicChannelSelect = MIC_CHANNEL_LEFT;
volatile int gMicGainShiftBits = kMicGainShiftDefaultBits;
volatile bool gCalibrationInProgress = false;

volatile bool gFftCaptureArmed = false;
volatile bool gFftFrameReady = false;
volatile bool gFftFramePending = false;
volatile float gFftDetectedHz = 0.0f;
volatile float gFftDetectedMagnitude = 0.0f;
volatile int gFftSampleCursor = 0;
float gFftInput[kFftLength];
float gFftOutput[kFftLength];
float gFftWindow[kFftLength];
bool gFftWindowReady = false;

i2s_config_t gI2sConfig = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = kSampleRateHz,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = kAudioBufferSamples,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
};

i2s_pin_config_t gI2sPins = {
    .bck_io_num = 5,
    .ws_io_num = 9,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = 10,
};

float clamp01(float value)
{
  if (value < 0.0f)
  {
    return 0.0f;
  }
  if (value > 1.0f)
  {
    return 1.0f;
  }
  return value;
}

float lerp(float a, float b, float t)
{
  return a + ((b - a) * t);
}

int32_t emaFilter(int32_t input, int32_t average, float alpha)
{
  const uint16_t integerAlpha = static_cast<uint16_t>(alpha * 65536.0f);
  const int64_t blended = (static_cast<int64_t>(input) * integerAlpha) +
                          (static_cast<int64_t>(average) * (65536 - integerAlpha));
  return static_cast<int32_t>((blended + 32768) / 65536);
}

void updateMallets()
{
  for (size_t i = 0; i < kMalletCount; ++i)
  {
    mallets[i].updateMallet();
  }
}

void prepareMalletsForCalibration()
{
  for (size_t i = 0; i < kMalletCount; ++i)
  {
    mallets[i].abortAndClearQueue();
  }
}

void serviceDelay(unsigned long delayMs)
{
  const unsigned long start = millis();
  while (millis() - start < delayMs)
  {
    updateMallets();
    delay(1);
  }
}

void updateMicrophoneEnvelope(int32_t sample)
{
  const int32_t absSample = (sample >= 0) ? sample : -sample;
  gFilteredMic = emaFilter(absSample, gFilteredMic, kMicEmaAlpha);

  int32_t peak = gDynamicMicPeak;
  if (peak > 4)
  {
    peak -= 4;
  }

  if (gFilteredMic > peak)
  {
    peak = gFilteredMic;
  }

  if (peak < 1)
  {
    peak = 1;
  }

  gDynamicMicPeak = peak;
  gNormalizedMic = static_cast<float>(gFilteredMic) / static_cast<float>(peak);
}

int32_t decodeMicSample(int32_t raw)
{
  // 24-bit MEMS microphones are typically left-aligned in a 32-bit I2S slot.
  // First align to signed 24-bit, then optionally trim gain for diagnostics.
  int32_t aligned24 = (raw >> kMicSlotTo24BitShiftBits);

  // SPH0645 has 18-bit effective precision; scrub undefined low bits so
  // tri-state tail/noise does not masquerade as real signal detail.
  aligned24 &= ~((1 << kMicUnusedLowBits) - 1);

  int gainShift = gMicGainShiftBits;
  if (gainShift < 0)
  {
    gainShift = 0;
  }
  if (gainShift > kMicGainShiftMaxBits)
  {
    gainShift = kMicGainShiftMaxBits;
  }

  return (aligned24 >> gainShift);
}

int frequencyToMidi(float frequencyHz)
{
  if (frequencyHz <= 0.0f)
  {
    return -1;
  }

  const float midiFloat = 69.0f + (12.0f * log2f(frequencyHz / 440.0f));
  const int midi = static_cast<int>(lroundf(midiFloat));
  if (midi < 0 || midi > 127)
  {
    return -1;
  }

  return midi+1;
}

void ensureFftWindow()
{
  if (gFftWindowReady)
  {
    return;
  }

  for (int i = 0; i < kFftLength; ++i)
  {
    gFftWindow[i] = 0.5f * (1.0f - cosf((2.0f * PI * static_cast<float>(i)) / static_cast<float>(kFftLength - 1)));
  }
  gFftWindowReady = true;
}

float fftMagnitudeAtBin(const fft_config_t *plan, int bin)
{
  const float real = plan->output[2 * bin];
  const float imag = plan->output[2 * bin + 1];
  return sqrtf((real * real) + (imag * imag));
}

void executeFftFrame()
{
  ensureFftWindow();
  for (int i = 0; i < kFftLength; ++i)
  {
    gFftInput[i] *= gFftWindow[i];
  }

  fft_config_t *plan = fft_init(kFftLength, FFT_REAL, FFT_FORWARD, gFftInput, gFftOutput);
  if (plan == nullptr)
  {
    gFftDetectedHz = 0.0f;
    gFftDetectedMagnitude = 0.0f;
    gFftFrameReady = true;
    return;
  }

  fft_execute(plan);

  const int minBin = static_cast<int>(ceilf(kFftMinDetectHz * static_cast<float>(kFftLength) / static_cast<float>(kSampleRateHz)));
  const int maxBin = static_cast<int>(floorf(kFftMaxDetectHz * static_cast<float>(kFftLength) / static_cast<float>(kSampleRateHz)));

  float bestMagnitude = 0.0f;
  int bestBin = minBin;

  for (int bin = minBin; bin <= maxBin && bin < (plan->size / 2); ++bin)
  {
    const float magnitude = fftMagnitudeAtBin(plan, bin);
    if (magnitude > bestMagnitude)
    {
      bestMagnitude = magnitude;
      bestBin = bin;
    }
  }

  float refinedBin = static_cast<float>(bestBin);
  if (bestBin > minBin && bestBin < maxBin)
  {
    const float m1 = fftMagnitudeAtBin(plan, bestBin - 1);
    const float m2 = bestMagnitude;
    const float m3 = fftMagnitudeAtBin(plan, bestBin + 1);
    const float denom = (m1 - (2.0f * m2) + m3);
    if (fabsf(denom) > 0.001f)
    {
      const float delta = 0.5f * (m1 - m3) / denom;
      if (delta > -1.0f && delta < 1.0f)
      {
        refinedBin = static_cast<float>(bestBin) + delta;
      }
    }
  }

  const float bestFrequency = refinedBin * static_cast<float>(kSampleRateHz) / static_cast<float>(kFftLength);
  gFftDetectedHz = bestFrequency;
  gFftDetectedMagnitude = bestMagnitude;
  gFftFrameReady = true;

  fft_destroy(plan);
}

void serviceFftAnalysis()
{
  if (gFftFramePending && !gFftFrameReady)
  {
    gFftFramePending = false;
    executeFftFrame();
  }
}

void processAudioBlock(int32_t *buffer, int32_t sampleCount)
{
  const bool stereoPairs = ((sampleCount % 2) == 0);
  if (stereoPairs)
  {
    for (int32_t i = 0; i < sampleCount; i += 2)
    {
      const int32_t leftDecoded = decodeMicSample(buffer[i]);
      const int32_t rightDecoded = decodeMicSample(buffer[i + 1]);
      const int32_t leftAbs = (leftDecoded >= 0) ? leftDecoded : -leftDecoded;
      const int32_t rightAbs = (rightDecoded >= 0) ? rightDecoded : -rightDecoded;

      if (leftAbs > gMicLeftPeak)
      {
        gMicLeftPeak = leftAbs;
      }
      if (rightAbs > gMicRightPeak)
      {
        gMicRightPeak = rightAbs;
      }
      if (gMicLeftPeak > 4)
      {
        gMicLeftPeak -= 4;
      }
      if (gMicRightPeak > 4)
      {
        gMicRightPeak -= 4;
      }

      const int32_t selected = (gMicChannelSelect == MIC_CHANNEL_RIGHT) ? rightDecoded : leftDecoded;
      gMicDc = emaFilter(selected, gMicDc, kMicDcAlpha);
      const int32_t sample = selected - gMicDc;
      updateMicrophoneEnvelope(sample);

      if (gFftCaptureArmed)
      {
        const int cursor = gFftSampleCursor;
        if (cursor < kFftLength)
        {
          gFftInput[cursor] = static_cast<float>(sample);
          gFftSampleCursor = cursor + 1;
        }

        if (gFftSampleCursor >= kFftLength)
        {
          gFftCaptureArmed = false;
          gFftFramePending = true;
          gFftSampleCursor = 0;
        }
      }
    }
    return;
  }

  // Fallback path if driver returns mono-shaped buffers.
  for (int32_t i = 0; i < sampleCount; ++i)
  {
    const int32_t decoded = decodeMicSample(buffer[i]);
    const int32_t absValue = (decoded >= 0) ? decoded : -decoded;
    if (absValue > gMicLeftPeak)
    {
      gMicLeftPeak = absValue;
    }
    if (gMicLeftPeak > 4)
    {
      gMicLeftPeak -= 4;
    }
    gMicDc = emaFilter(decoded, gMicDc, kMicDcAlpha);
    const int32_t sample = decoded - gMicDc;
    updateMicrophoneEnvelope(sample);

    if (gFftCaptureArmed)
    {
      const int cursor = gFftSampleCursor;
      if (cursor < kFftLength)
      {
        gFftInput[cursor] = static_cast<float>(sample);
        gFftSampleCursor = cursor + 1;
      }

      if (gFftSampleCursor >= kFftLength)
      {
        gFftCaptureArmed = false;
        gFftFramePending = true;
        gFftSampleCursor = 0;
      }
    }
  }
}

void i2sAudioWriterTask(void *param)
{
  I2SSampler *sampler = static_cast<I2SSampler *>(param);
  const TickType_t maxWait = pdMS_TO_TICKS(300);

  while (true)
  {
    const uint32_t notified = ulTaskNotifyTake(pdTRUE, maxWait);
    if (notified > 0)
    {
      int32_t *audioBuffer = sampler->getCapturedAudioBuffer();
      const int32_t sampleCount = sampler->getBufferSizeInBytes() / static_cast<int32_t>(sizeof(int32_t));
      processAudioBlock(audioBuffer, sampleCount);
    }
  }
}

void setupAudioCapture()
{
  gSampler = new I2SMEMSSampler(gI2sPins, true);
  xTaskCreatePinnedToCore(i2sAudioWriterTask, "I2S Writer Task", 8192, gSampler, 1, &gAudioWriterTaskHandle, 1);
  gSampler->start(I2S_NUM_1, gI2sConfig, kAudioBufferSamples * static_cast<int32_t>(sizeof(int32_t)), gAudioWriterTaskHandle);
}

unsigned long estimateNoteEventMillis(uint16_t timestamp)
{
  static unsigned long lastRolloverMillis = 0;
  static unsigned long lastRolloverTimestamp = 0;
  static unsigned long lastTimestamp = kBleTimestampRolloverMs + 1;

  if (timestamp < lastTimestamp)
  {
    lastRolloverMillis = millis();
    lastRolloverTimestamp = timestamp;
  }

  const unsigned long now = millis();
  const unsigned long elapsedSinceRollover = now - lastRolloverMillis;
  const unsigned long intervalStart = now - elapsedSinceRollover - lastRolloverTimestamp;
  const unsigned long rolloverCount = (now - intervalStart) / kBleTimestampRolloverMs;

  lastTimestamp = timestamp;
  return intervalStart + (rolloverCount * kBleTimestampRolloverMs) + timestamp;
}

void configureProbeStrike(Mallet &mallet, int strikePower)
{
  mallet.strikePower = constrain(strikePower, 250, 1023);
  mallet.strikeDuration = 420;
  mallet.coastPower = 0;
  mallet.coastDuration = 1;
  mallet.reboundPower = 0;
  mallet.reboundDuration = 1;
}

void applyMalletTimingFromLag(Mallet &mallet, unsigned long lagMs)
{
  const unsigned long strikeDuration = constrain(static_cast<unsigned long>(lagMs * 1.5f), 100UL, 1600UL);
  const unsigned long coastDuration = constrain(static_cast<unsigned long>(lagMs * 0.10f), 0UL, 300UL);
  const unsigned long reboundDuration = constrain(static_cast<unsigned long>(lagMs * 1.4f), 120UL, 2000UL);

  mallet.strikePower = 1023;
  mallet.strikeDuration = static_cast<int>(strikeDuration);
  mallet.coastPower = 0;
  mallet.coastDuration = static_cast<int>(coastDuration);
  mallet.reboundPower = 100;
  mallet.reboundDuration = static_cast<int>(reboundDuration);
}

void configurePitchProbeStrike(Mallet &mallet, const VelocityCalibrationModel &model)
{
  const unsigned long lagForTiming = model.valid ? static_cast<unsigned long>(model.hardLagMs) : 140UL;
  applyMalletTimingFromLag(mallet, lagForTiming);
  mallet.strikePower = constrain(static_cast<int>(model.hardPower), 250, 1023);
}

int32_t sampleAmbientMic(unsigned long windowMs)
{
  int32_t ambientPeak = 0;
  const unsigned long start = millis();
  while (millis() - start < windowMs)
  {
    const int32_t current = gFilteredMic;
    if (current > ambientPeak)
    {
      ambientPeak = current;
    }
    updateMallets();
    delay(1);
  }
  return ambientPeak;
}

struct AmbientWindowStats
{
  int32_t peak;
  int32_t minValue;
  int32_t maxValue;
  int32_t peakToPeak;
};

AmbientWindowStats sampleAmbientWindow(unsigned long windowMs)
{
  AmbientWindowStats stats = {};
  stats.peak = 0;
  stats.minValue = INT32_MAX;
  stats.maxValue = INT32_MIN;

  const unsigned long start = millis();
  while (millis() - start < windowMs)
  {
    const int32_t current = gFilteredMic;
    if (current > stats.peak)
    {
      stats.peak = current;
    }
    if (current < stats.minValue)
    {
      stats.minValue = current;
    }
    if (current > stats.maxValue)
    {
      stats.maxValue = current;
    }
    updateMallets();
    delay(1);
  }

  if (stats.maxValue >= stats.minValue)
  {
    stats.peakToPeak = stats.maxValue - stats.minValue;
  }
  else
  {
    stats.peakToPeak = 0;
  }

  return stats;
}

void waitForRingdown(unsigned long timeoutMs)
{
  const unsigned long start = millis();
  unsigned long stableStart = 0;
  int32_t runningFloor = gFilteredMic;
  while (millis() - start < timeoutMs)
  {
    const int32_t mic = gFilteredMic;
    if (mic < runningFloor)
    {
      runningFloor = mic;
    }

    const int32_t quietThreshold = runningFloor + kRingdownThresholdRise;
    if (mic <= quietThreshold)
    {
      if (stableStart == 0)
      {
        stableStart = millis();
      }
      if (millis() - stableStart >= kRingdownStableMs)
      {
        return;
      }
    }
    else
    {
      stableStart = 0;
    }

    updateMallets();
    delay(1);
  }
}

bool waitForMalletIdle(Mallet &mallet, unsigned long timeoutMs)
{
  const unsigned long start = millis();
  while (!mallet.isIdle() && (millis() - start < timeoutMs))
  {
    updateMallets();
    delay(1);
  }
  return mallet.isIdle();
}

StrikeMeasurement measureStrikeResponse(Mallet &mallet, int32_t thresholdRise, unsigned long maxImpactLagMs, int32_t minRiseRequired)
{
  StrikeMeasurement result = {
    kCalibrationImpactTimeoutMs,
    0,
    false,
    gFilteredMic,
    gFilteredMic + thresholdRise,
    0,
    minRiseRequired,
  };

  if (!waitForMalletIdle(mallet, kCalibrationMalletIdleTimeoutMs))
  {
    return result;
  }
  waitForRingdown(kRingdownTimeoutMs);
  if (!waitForMalletIdle(mallet, kCalibrationMalletIdleTimeoutMs))
  {
    return result;
  }
  const AmbientWindowStats ambientWindow = sampleAmbientWindow(120);
  const int32_t ambient = ambientWindow.peak;
  int32_t adaptiveRise = thresholdRise;
  const int32_t noiseRise = (ambientWindow.peakToPeak * kImpactNoiseMultiplier) + kImpactNoiseRiseFloor;
  if (noiseRise > adaptiveRise)
  {
    adaptiveRise = noiseRise;
  }
  const int32_t triggerThreshold = ambient + adaptiveRise;
  const int32_t rearmThreshold = triggerThreshold - kImpactThresholdHysteresis;

  result = {
    maxImpactLagMs,
    0,
    false,
    ambient,
    triggerThreshold,
    ambientWindow.peakToPeak,
    minRiseRequired,
  };

  int32_t detectWindowPeak = ambient;
  const unsigned long startMicros = micros();
  mallet.triggerMallet();
  bool crossedBelowThreshold = (gFilteredMic <= rearmThreshold);

  while ((micros() - startMicros) < (kCalibrationImpactTimeoutMs * 1000UL))
  {
    const int32_t mic = gFilteredMic;
    const unsigned long elapsedMs = (micros() - startMicros) / 1000UL;
    if (elapsedMs <= maxImpactLagMs && mic > detectWindowPeak)
    {
      detectWindowPeak = mic;
    }

    if (mic <= rearmThreshold)
    {
      crossedBelowThreshold = true;
    }

    if (!result.impactDetected &&
        elapsedMs >= kImpactDetectGuardMs &&
        elapsedMs <= maxImpactLagMs &&
        crossedBelowThreshold &&
        mic >= triggerThreshold)
    {
      result.impactDetected = true;
      result.lagMs = elapsedMs;
    }

    updateMallets();
    delay(1);
  }

  result.peakRise = (detectWindowPeak > ambient) ? (detectWindowPeak - ambient) : 0;
  if (result.impactDetected && result.peakRise < minRiseRequired)
  {
    result.impactDetected = false;
    result.lagMs = maxImpactLagMs;
  }
  return result;
}

int detectPitchFromStrike(Mallet &mallet, unsigned long captureDelayMs, float *detectedHz, float *detectedMagnitude)
{
  waitForMalletIdle(mallet, kCalibrationMalletIdleTimeoutMs);
  gFftFrameReady = false;
  gFftFramePending = false;
  gFftSampleCursor = 0;
  gFftCaptureArmed = false;

  mallet.triggerMallet();

  const unsigned long captureStart = millis();
  while (millis() - captureStart < captureDelayMs)
  {
    updateMallets();
    delay(1);
  }

  gFftFrameReady = false;
  gFftFramePending = false;
  gFftSampleCursor = 0;
  gFftCaptureArmed = true;

  const unsigned long start = millis();
  while (!gFftFrameReady && (millis() - start < kFftTimeoutMs))
  {
    serviceFftAnalysis();
    updateMallets();
    delay(1);
  }

  gFftCaptureArmed = false;

  if (!gFftFrameReady)
  {
    if (detectedHz != nullptr)
    {
      *detectedHz = 0.0f;
    }
    if (detectedMagnitude != nullptr)
    {
      *detectedMagnitude = 0.0f;
    }
    return -1;
  }

  const float frequency = gFftDetectedHz;
  const float magnitude = gFftDetectedMagnitude;

  if (detectedHz != nullptr)
  {
    *detectedHz = frequency;
  }
  if (detectedMagnitude != nullptr)
  {
    *detectedMagnitude = magnitude;
  }

  if (magnitude < kFftMinMagnitude)
  {
    return -1;
  }

  return frequencyToMidi(frequency);
}

void setVelocityCalibrationDefaults(size_t index)
{
  gVelocityCalibration[index].softPower = 460;
  gVelocityCalibration[index].hardPower = 1023;
  gVelocityCalibration[index].softLagMs = 420;
  gVelocityCalibration[index].hardLagMs = 300;
  gVelocityCalibration[index].softImpact = 1;
  gVelocityCalibration[index].hardImpact = 2;
  gVelocityCalibration[index].valid = false;
}

void initializeVelocityCalibrationDefaults()
{
  for (size_t i = 0; i < kMalletCount; ++i)
  {
    setVelocityCalibrationDefaults(i);
  }
}

bool calibrateVelocityForMallet(size_t index)
{
  Mallet &mallet = mallets[index];
  uint16_t softPower = kSoftPowerInitial;
  const uint16_t hardPower = 1023;
  const int repeats = kCalibrationRepeats;

  unsigned long softLagSum = 0;
  unsigned long hardLagSum = 0;
  uint32_t softImpactSum = 0;
  uint32_t hardImpactSum = 0;
  uint32_t softPowerSum = 0;
  int softHits = 0;
  int hardHits = 0;

  for (int r = 0; r < repeats; ++r)
  {
    configureProbeStrike(mallet, hardPower);
    StrikeMeasurement hard = measureStrikeResponse(mallet, kImpactThresholdHardRise, kCalibrationHardMaxLagMs, kHardMinRiseFloor);
    Serial.print("    hard: ambient=");
    Serial.print(hard.ambient);
    Serial.print(" threshold=");
    Serial.print(hard.triggerThreshold);
    Serial.print(" rise=");
    Serial.print(hard.peakRise);
    Serial.print(" minRise=");
    Serial.print(hard.minRiseRequired);
    Serial.print(" noise=");
    Serial.print(hard.noiseP2P);
    Serial.print(" lag=");
    Serial.print(hard.lagMs);
    Serial.print(" hit=");
    Serial.println(hard.impactDetected ? "yes" : "no");
    if (hard.impactDetected)
    {
      hardImpactSum += static_cast<uint32_t>(hard.peakRise);
      hardLagSum += hard.lagMs;
      hardHits++;
    }

    serviceDelay(kCalibrationInterStrikeDelayMs);
    waitForRingdown(kRingdownTimeoutMs);

    unsigned long softMaxLagMs = kCalibrationSoftMaxLagFloorMs;
    int32_t softMinRise = kSoftMinRiseFloor;
    if (hard.impactDetected)
    {
      softMaxLagMs = hard.lagMs + kCalibrationSoftLagSlackMs;
      if (softMaxLagMs < kCalibrationSoftMaxLagFloorMs)
      {
        softMaxLagMs = kCalibrationSoftMaxLagFloorMs;
      }
      const int32_t derivedSoftMinRise = hard.peakRise / kSoftMinRiseHardDivisor;
      if (derivedSoftMinRise > softMinRise)
      {
        softMinRise = derivedSoftMinRise;
      }
    }

    StrikeMeasurement soft = {};
    uint16_t selectedSoftPower = softPower;
    bool softAccepted = false;
    for (int attempt = 0; attempt < kSoftPowerAttempts; ++attempt)
    {
      const uint16_t candidatePower = static_cast<uint16_t>(constrain(static_cast<int>(softPower) + (attempt * static_cast<int>(kSoftPowerStep)),
                                                                       static_cast<int>(kSoftPowerInitial),
                                                                       static_cast<int>(kSoftPowerMax)));
      configureProbeStrike(mallet, candidatePower);
      soft = measureStrikeResponse(mallet, kImpactThresholdSoftRise, softMaxLagMs, softMinRise);

      Serial.print("    soft: pwr=");
      Serial.print(candidatePower);
      Serial.print(" ambient=");
      Serial.print(soft.ambient);
      Serial.print(" threshold=");
      Serial.print(soft.triggerThreshold);
      Serial.print(" rise=");
      Serial.print(soft.peakRise);
      Serial.print(" minRise=");
      Serial.print(soft.minRiseRequired);
      Serial.print(" noise=");
      Serial.print(soft.noiseP2P);
      Serial.print(" lag=");
      Serial.print(soft.lagMs);
      Serial.print(" hit=");
      Serial.println(soft.impactDetected ? "yes" : "no");

      if (soft.impactDetected)
      {
        selectedSoftPower = candidatePower;
        softAccepted = true;
        break;
      }

      serviceDelay(kCalibrationSoftRetryDelayMs);
      waitForRingdown(kRingdownTimeoutMs);
    }

    if (softAccepted)
    {
      softPower = selectedSoftPower;
      softPowerSum += static_cast<uint32_t>(selectedSoftPower);
      softImpactSum += static_cast<uint32_t>(soft.peakRise);
      softLagSum += soft.lagMs;
      softHits++;
    }

    serviceDelay(kCalibrationInterStrikeDelayMs);
    waitForRingdown(kRingdownTimeoutMs);
  }

  VelocityCalibrationModel model;
  model.softPower = (softHits > 0) ? static_cast<uint16_t>(softPowerSum / softHits) : static_cast<uint16_t>(constrain(static_cast<int>(kSoftPowerInitial + (2 * kSoftPowerStep)), 250, static_cast<int>(hardPower)));
  model.hardPower = hardPower;
  model.hardLagMs = (hardHits > 0) ? static_cast<uint16_t>(hardLagSum / hardHits) : 320;
  model.softLagMs = (softHits > 0) ? static_cast<uint16_t>(softLagSum / softHits) : static_cast<uint16_t>(model.hardLagMs + 80);
  if (model.softLagMs < model.hardLagMs)
  {
    model.softLagMs = model.hardLagMs;
  }

  model.softImpact = softImpactSum / repeats;
  model.hardImpact = hardImpactSum / repeats;
  if (model.hardImpact <= model.softImpact)
  {
    model.hardImpact = model.softImpact + 1;
  }

  model.valid = (hardHits > 0);
  gVelocityCalibration[index] = model;

  const unsigned long defaultLag = static_cast<unsigned long>((static_cast<uint32_t>(model.softLagMs) + static_cast<uint32_t>(model.hardLagMs)) / 2UL);
  mallet.setDelay(defaultLag);
  applyMalletTimingFromLag(mallet, defaultLag);

  Serial.print("  velocity: softLag=");
  Serial.print(model.softLagMs);
  Serial.print(" hardLag=");
  Serial.print(model.hardLagMs);
  Serial.print(" softImpact=");
  Serial.print(model.softImpact);
  Serial.print(" hardImpact=");
  Serial.print(model.hardImpact);
  Serial.print(" valid=");
  Serial.println(model.valid ? "yes" : "no");

  return model.valid;
}

ScheduledStrike buildStrikeForVelocity(size_t malletIndex, uint8_t velocity)
{
  VelocityCalibrationModel model = gVelocityCalibration[malletIndex];
  if (!model.valid)
  {
    setVelocityCalibrationDefaults(malletIndex);
    model = gVelocityCalibration[malletIndex];
  }

  const float velocityNorm = clamp01((static_cast<float>(velocity) - 1.0f) / 126.0f);
  const float shaped = powf(velocityNorm, 1.25f);

  const float lagF = lerp(static_cast<float>(model.softLagMs), static_cast<float>(model.hardLagMs), shaped);
  const float powerF = lerp(static_cast<float>(model.softPower), static_cast<float>(model.hardPower), shaped);

  ScheduledStrike strike = {};
  strike.lagMs = static_cast<unsigned long>(lagF);

  strike.profile.strikePower = constrain(static_cast<int>(lroundf(powerF)), 250, 1023);
  strike.profile.strikeDuration = constrain(static_cast<int>(lroundf((lagF * 1.50f) + ((1.0f - shaped) * 120.0f))), 90, 1600);
  strike.profile.coastPower = 0;
  strike.profile.coastDuration = constrain(static_cast<int>(lroundf(lagF * 0.10f)), 0, 300);

  // Keep positive rebound torque so the actuator actively catches the return.
  strike.profile.reboundPower = constrain(static_cast<int>(lroundf(70.0f + (90.0f * shaped))), 40, 220);
  strike.profile.reboundDuration = constrain(static_cast<int>(lroundf((lagF * 1.40f) + ((1.0f - shaped) * 90.0f))), 100, 2000);

  return strike;
}

void scheduleNote(size_t malletIndex, unsigned long targetNoteMillis, uint8_t velocity)
{
  ScheduledStrike strike = buildStrikeForVelocity(malletIndex, velocity);

  const unsigned long now = millis();
  const unsigned long strikeAtMillis = (targetNoteMillis > strike.lagMs) ? (targetNoteMillis - strike.lagMs) : 0;
  const unsigned long delayMs = (strikeAtMillis > now) ? (strikeAtMillis - now) : 0;

  mallets[malletIndex].delayedTrigger(delayMs, strike.profile);
}

CalibrationEntry buildCalibrationEntry(size_t index)
{
  CalibrationEntry entry = {};
  entry.midiPitch = static_cast<int16_t>(mallets[index].getMidiPitch());
  entry.softPower = gVelocityCalibration[index].softPower;
  entry.hardPower = gVelocityCalibration[index].hardPower;
  entry.softLagMs = gVelocityCalibration[index].softLagMs;
  entry.hardLagMs = gVelocityCalibration[index].hardLagMs;
  entry.softImpact = gVelocityCalibration[index].softImpact;
  entry.hardImpact = gVelocityCalibration[index].hardImpact;
  entry.valid = gVelocityCalibration[index].valid ? 1 : 0;
  return entry;
}

void applyCalibrationEntry(size_t index, const CalibrationEntry &entry)
{
  if (entry.valid != 0)
  {
    gVelocityCalibration[index].softPower = entry.softPower;
    gVelocityCalibration[index].hardPower = entry.hardPower;
    gVelocityCalibration[index].softLagMs = entry.softLagMs;
    gVelocityCalibration[index].hardLagMs = entry.hardLagMs;
    gVelocityCalibration[index].softImpact = entry.softImpact;
    gVelocityCalibration[index].hardImpact = entry.hardImpact;
    gVelocityCalibration[index].valid = true;
  }
  else
  {
    setVelocityCalibrationDefaults(index);
  }

  if (entry.midiPitch >= 0 && entry.midiPitch <= 127)
  {
    mallets[index].setMidiPitch(entry.midiPitch);
  }

  const unsigned long defaultLag = static_cast<unsigned long>((static_cast<uint32_t>(gVelocityCalibration[index].softLagMs) +
                                    static_cast<uint32_t>(gVelocityCalibration[index].hardLagMs)) / 2UL);
  mallets[index].setDelay(defaultLag);
  applyMalletTimingFromLag(mallets[index], defaultLag);
}

bool saveCalibrationToEeprom()
{
  CalibrationHeader header = {};
  EEPROM.get(kCalibrationHeaderAddress, header);

  if (header.magic != kCalibrationMagic || header.version != kCalibrationVersion || header.malletCount != kMalletCount)
  {
    header.writeCount = 0;
  }

  header.magic = kCalibrationMagic;
  header.version = kCalibrationVersion;
  header.malletCount = kMalletCount;
  header.writeCount += 1;

  EEPROM.put(kCalibrationHeaderAddress, header);
  for (size_t i = 0; i < kMalletCount; ++i)
  {
    const size_t address = kCalibrationEntriesAddress + (i * sizeof(CalibrationEntry));
    CalibrationEntry entry = buildCalibrationEntry(i);
    EEPROM.put(address, entry);
  }

  const bool committed = EEPROM.commit();
  Serial.print("Saved calibration, writeCount=");
  Serial.print(header.writeCount);
  Serial.print(", status=");
  Serial.println(committed ? "ok" : "failed");
  return committed;
}

bool loadCalibrationFromEeprom()
{
  CalibrationHeader header = {};
  EEPROM.get(kCalibrationHeaderAddress, header);

  if (header.magic != kCalibrationMagic || header.version != kCalibrationVersion || header.malletCount != kMalletCount)
  {
    return false;
  }

  for (size_t i = 0; i < kMalletCount; ++i)
  {
    CalibrationEntry entry = {};
    const size_t address = kCalibrationEntriesAddress + (i * sizeof(CalibrationEntry));
    EEPROM.get(address, entry);
    applyCalibrationEntry(i, entry);
  }

  Serial.print("Loaded calibration from EEPROM writes: ");
  Serial.println(header.writeCount);
  return true;
}

void printMalletStatus()
{
  Serial.println("Mallet status:");
  for (size_t i = 0; i < kMalletCount; ++i)
  {
    Serial.print("  #");
    Serial.print(i);
    Serial.print(" pitch=");
    Serial.print(mallets[i].getMidiPitch());
    Serial.print(" lagSoft=");
    Serial.print(gVelocityCalibration[i].softLagMs);
    Serial.print(" lagHard=");
    Serial.print(gVelocityCalibration[i].hardLagMs);
    Serial.print(" pwrSoft=");
    Serial.print(gVelocityCalibration[i].softPower);
    Serial.print(" pwrHard=");
    Serial.print(gVelocityCalibration[i].hardPower);
    Serial.print(" impactSoft=");
    Serial.print(gVelocityCalibration[i].softImpact);
    Serial.print(" impactHard=");
    Serial.print(gVelocityCalibration[i].hardImpact);
    Serial.print(" valid=");
    Serial.println(gVelocityCalibration[i].valid ? "yes" : "no");
  }
}

void printAudioStatus()
{
  Serial.print("Audio filtered=");
  Serial.print(gFilteredMic);
  Serial.print(" peak=");
  Serial.print(gDynamicMicPeak);
  Serial.print(" norm=");
  Serial.print(gNormalizedMic, 3);
  Serial.print(" fftArmed=");
  Serial.print(gFftCaptureArmed ? "yes" : "no");
  Serial.print(" fftReady=");
  Serial.print(gFftFrameReady ? "yes" : "no");
  Serial.print(" fftHz=");
  Serial.print(gFftDetectedHz, 2);
  Serial.print(" fftMag=");
  Serial.print(gFftDetectedMagnitude, 2);
  Serial.print(" micCh=");
  Serial.print((gMicChannelSelect == MIC_CHANNEL_RIGHT) ? "right" : "left");
  Serial.print(" leftPk=");
  Serial.print(gMicLeftPeak);
  Serial.print(" rightPk=");
  Serial.print(gMicRightPeak);
  Serial.print(" dc=");
  Serial.println(gMicDc);
}

void printMicDiagnostics()
{
  Serial.print("micdiag channel=");
  Serial.print((gMicChannelSelect == MIC_CHANNEL_RIGHT) ? "right" : "left");
  Serial.print(" gainShift=");
  Serial.print(gMicGainShiftBits);
  Serial.print(" decodeShift=");
  Serial.print(kMicSlotTo24BitShiftBits);
  Serial.print(" leftPeak=");
  Serial.print(gMicLeftPeak);
  Serial.print(" rightPeak=");
  Serial.print(gMicRightPeak);
  Serial.print(" filtered=");
  Serial.print(gFilteredMic);
  Serial.print(" dc=");
  Serial.println(gMicDc);
}

void runMicProbe(const String &cmd)
{
  unsigned long windowMs = 400;
  const int separator = cmd.indexOf(' ');
  if (separator >= 0 && separator + 1 < static_cast<int>(cmd.length()))
  {
    const long parsed = cmd.substring(separator + 1).toInt();
    if (parsed > 0)
    {
      windowMs = static_cast<unsigned long>(parsed);
    }
  }

  if (windowMs < 100)
  {
    windowMs = 100;
  }
  if (windowMs > 3000)
  {
    windowMs = 3000;
  }

  gMicLeftPeak = 0;
  gMicRightPeak = 0;

  int32_t filteredMin = INT32_MAX;
  int32_t filteredMax = INT32_MIN;
  int32_t dcMin = INT32_MAX;
  int32_t dcMax = INT32_MIN;
  float normMax = 0.0f;

  const unsigned long start = millis();
  while (millis() - start < windowMs)
  {
    const int32_t filtered = gFilteredMic;
    const int32_t dc = gMicDc;
    const float norm = gNormalizedMic;

    if (filtered < filteredMin)
    {
      filteredMin = filtered;
    }
    if (filtered > filteredMax)
    {
      filteredMax = filtered;
    }
    if (dc < dcMin)
    {
      dcMin = dc;
    }
    if (dc > dcMax)
    {
      dcMax = dc;
    }
    if (norm > normMax)
    {
      normMax = norm;
    }

    updateMallets();
    delay(1);
  }

  const int32_t filteredP2P = (filteredMax > filteredMin) ? (filteredMax - filteredMin) : 0;

  Serial.print("micprobe ms=");
  Serial.print(windowMs);
  Serial.print(" ch=");
  Serial.print((gMicChannelSelect == MIC_CHANNEL_RIGHT) ? "right" : "left");
  Serial.print(" leftPeak=");
  Serial.print(gMicLeftPeak);
  Serial.print(" rightPeak=");
  Serial.print(gMicRightPeak);
  Serial.print(" filtMin=");
  Serial.print(filteredMin);
  Serial.print(" filtMax=");
  Serial.print(filteredMax);
  Serial.print(" filtP2P=");
  Serial.print(filteredP2P);
  Serial.print(" dcMin=");
  Serial.print(dcMin);
  Serial.print(" dcMax=");
  Serial.print(dcMax);
  Serial.print(" normMax=");
  Serial.println(normMax, 3);
}

void setMicChannelFromCommand(const String &cmd)
{
  int channel = -1;
  if (cmd == "micch left" || cmd == "micch l" || cmd == "micch 0")
  {
    channel = MIC_CHANNEL_LEFT;
  }
  else if (cmd == "micch right" || cmd == "micch r" || cmd == "micch 1")
  {
    channel = MIC_CHANNEL_RIGHT;
  }

  if (channel < 0)
  {
    Serial.println("micch usage: micch left|right (or 0|1)");
    return;
  }

  gMicChannelSelect = channel;
  Serial.print("Mic channel set to ");
  Serial.println((gMicChannelSelect == MIC_CHANNEL_RIGHT) ? "right" : "left");
}

void setMicShiftFromCommand(const String &cmd)
{
  const int separator = cmd.indexOf(' ');
  if (separator < 0 || separator + 1 >= static_cast<int>(cmd.length()))
  {
    Serial.print("micshift usage: micshift <0..");
    Serial.print(kMicGainShiftMaxBits);
    Serial.println(">");
    return;
  }

  int shift = cmd.substring(separator + 1).toInt();
  if (shift < 0)
  {
    shift = 0;
  }
  if (shift > kMicGainShiftMaxBits)
  {
    shift = kMicGainShiftMaxBits;
  }

  gMicGainShiftBits = shift;
  gMicLeftPeak = 0;
  gMicRightPeak = 0;
  gMicDc = 0;
  Serial.print("Mic gain shift set to ");
  Serial.print(gMicGainShiftBits);
  Serial.print(" (24-bit decode shift fixed at ");
  Serial.print(kMicSlotTo24BitShiftBits);
  Serial.println(")");
}

void runFftTestForMallet(int index)
{
  if (index < 0 || index >= static_cast<int>(kMalletCount))
  {
    Serial.print("ffttest mallet index out of range 0..");
    Serial.println(static_cast<int>(kMalletCount) - 1);
    return;
  }

  const size_t malletIndex = static_cast<size_t>(index);
  Mallet &mallet = mallets[malletIndex];
  configurePitchProbeStrike(mallet, gVelocityCalibration[malletIndex]);
  waitForRingdown(kRingdownTimeoutMs);
  serviceDelay(kCalibrationInterStrikeDelayMs);

  float detectedHz = 0.0f;
  float detectedMagnitude = 0.0f;
  unsigned long captureDelayMs = 120;
  if (gVelocityCalibration[malletIndex].valid)
  {
    captureDelayMs = static_cast<unsigned long>(gVelocityCalibration[malletIndex].hardLagMs + kPitchCaptureOffsetMs);
  }
  const int detectedPitch = detectPitchFromStrike(mallet, captureDelayMs, &detectedHz, &detectedMagnitude);

  Serial.print("ffttest mallet=");
  Serial.print(index);
  Serial.print(" hz=");
  Serial.print(detectedHz, 2);
  Serial.print(" mag=");
  Serial.print(detectedMagnitude, 2);
  Serial.print(" midi=");
  Serial.println(detectedPitch);
}

void playCalibrationArpeggio()
{
  Serial.println("Calibration arpeggio");
  size_t order[kMalletCount] = {};
  for (size_t i = 0; i < kMalletCount; ++i)
  {
    order[i] = i;
  }

  // Sort mallets by detected MIDI pitch so arpeggio is truly by pitch.
  for (size_t i = 0; i + 1 < kMalletCount; ++i)
  {
    size_t best = i;
    for (size_t j = i + 1; j < kMalletCount; ++j)
    {
      if (mallets[order[j]].getMidiPitch() < mallets[order[best]].getMidiPitch())
      {
        best = j;
      }
    }
    if (best != i)
    {
      const size_t tmp = order[i];
      order[i] = order[best];
      order[best] = tmp;
    }
  }

  auto playArpStep = [&](size_t malletIndex) {
    Mallet &mallet = mallets[malletIndex];
    waitForMalletIdle(mallet, kCalibrationMalletIdleTimeoutMs);

    const uint16_t power = gVelocityCalibration[malletIndex].valid ? gVelocityCalibration[malletIndex].softPower : 700;
    configureProbeStrike(mallet, power);
    mallet.triggerMallet();

    Serial.print("  arp #");
    Serial.print(malletIndex);
    Serial.print(" midi=");
    Serial.println(mallet.getMidiPitch());

    serviceDelay(kCalibrationInterMalletDelayMs);
  };

  for (size_t i = 0; i < kMalletCount; ++i)
  {
    playArpStep(order[i]);
  }

  for (int i = static_cast<int>(kMalletCount) - 2; i >= 0; --i)
  {
    playArpStep(order[static_cast<size_t>(i)]);
  }
}

void runVelocityCalibrationOnly()
{
  gCalibrationInProgress = true;
  prepareMalletsForCalibration();
  Serial.println("Starting velocity calibration");
  for (size_t i = 0; i < kMalletCount; ++i)
  {
    mallets[i].abortAndClearQueue();
    Serial.print("Calibrating velocity for mallet #");
    Serial.println(i);
    calibrateVelocityForMallet(i);
    waitForRingdown(kRingdownTimeoutMs);
    serviceDelay(kCalibrationInterMalletDelayMs);
  }
  saveCalibrationToEeprom();
  printMalletStatus();
  Serial.println("Velocity calibration complete");
  prepareMalletsForCalibration();
  gCalibrationInProgress = false;
}

void runFullCalibration()
{
  gCalibrationInProgress = true;
  prepareMalletsForCalibration();
  Serial.println("Starting full calibration");

  for (size_t i = 0; i < kMalletCount; ++i)
  {
    mallets[i].abortAndClearQueue();
    Serial.print("Calibrating mallet #");
    Serial.println(i);

    calibrateVelocityForMallet(i);

    // Detect pitch using calibrated hard-strike timing/power.
    configurePitchProbeStrike(mallets[i], gVelocityCalibration[i]);
    waitForRingdown(kRingdownTimeoutMs);
    serviceDelay(kCalibrationInterStrikeDelayMs);

    float detectedHz = 0.0f;
    float detectedMagnitude = 0.0f;
    unsigned long captureDelayMs = static_cast<unsigned long>(gVelocityCalibration[i].hardLagMs + kPitchCaptureOffsetMs);
    const int detectedPitch = detectPitchFromStrike(mallets[i], captureDelayMs, &detectedHz, &detectedMagnitude);
    if (detectedPitch >= 0)
    {
      mallets[i].setMidiPitch(detectedPitch);
    }

    Serial.print("  pitchHz=");
    Serial.print(detectedHz);
    Serial.print(" mag=");
    Serial.print(detectedMagnitude);
    Serial.print(" detected=");
    Serial.print(detectedPitch);
    Serial.print(" midi=");
    Serial.println(mallets[i].getMidiPitch());

    waitForRingdown(kRingdownTimeoutMs);
    serviceDelay(kCalibrationInterMalletDelayMs);
  }

  playCalibrationArpeggio();
  saveCalibrationToEeprom();
  printMalletStatus();
  Serial.println("Full calibration complete");
  prepareMalletsForCalibration();
  gCalibrationInProgress = false;
}

void setMalletMidiFromCommand(const String &cmd)
{
  const int firstSpace = cmd.indexOf(' ');
  if (firstSpace < 0)
  {
    Serial.println("setnote usage: setnote <malletIndex 0..9> <midi 0..127>");
    return;
  }

  const int secondSpace = cmd.indexOf(' ', firstSpace + 1);
  if (secondSpace < 0)
  {
    Serial.println("setnote usage: setnote <malletIndex 0..9> <midi 0..127>");
    return;
  }

  const int malletIndex = cmd.substring(firstSpace + 1, secondSpace).toInt();
  const int midiNote = cmd.substring(secondSpace + 1).toInt();

  if (malletIndex < 0 || malletIndex >= static_cast<int>(kMalletCount))
  {
    Serial.print("setnote mallet index out of range 0..");
    Serial.println(static_cast<int>(kMalletCount) - 1);
    return;
  }

  if (midiNote < 0 || midiNote > 127)
  {
    Serial.println("setnote midi out of range 0..127");
    return;
  }

  mallets[static_cast<size_t>(malletIndex)].setMidiPitch(midiNote);
  const bool saved = saveCalibrationToEeprom();

  Serial.print("setnote #");
  Serial.print(malletIndex);
  Serial.print(" -> ");
  Serial.print(midiNote);
  Serial.print(" (saved=");
  Serial.print(saved ? "yes" : "no");
  Serial.println(")");
}

void handleSerialCommands()
{
  if (!Serial.available())
  {
    return;
  }

  const String command = Serial.readStringUntil('\n');
  String cmd = command;
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "cal")
  {
    runFullCalibration();
  }
  else if (cmd == "calvel")
  {
    runVelocityCalibrationOnly();
  }
  else if (cmd == "status")
  {
    printMalletStatus();
  }
  else if (cmd == "audiostatus")
  {
    printAudioStatus();
  }
  else if (cmd == "micdiag")
  {
    printMicDiagnostics();
  }
  else if (cmd.startsWith("micprobe"))
  {
    runMicProbe(cmd);
  }
  else if (cmd.startsWith("micch"))
  {
    setMicChannelFromCommand(cmd);
  }
  else if (cmd.startsWith("micshift"))
  {
    setMicShiftFromCommand(cmd);
  }
  else if (cmd.startsWith("ffttest"))
  {
    int malletIndex = 0;
    const int separator = cmd.indexOf(' ');
    if (separator >= 0 && separator + 1 < static_cast<int>(cmd.length()))
    {
      malletIndex = cmd.substring(separator + 1).toInt();
    }
    runFftTestForMallet(malletIndex);
  }
  else if (cmd.startsWith("setnote"))
  {
    setMalletMidiFromCommand(cmd);
  }
}

void handleNoteOn(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp)
{
  (void)channel;
  if (velocity == 0 || gCalibrationInProgress)
  {
    return;
  }

  const unsigned long noteMillis = estimateNoteEventMillis(timestamp) + kMidiLeadTimeMs;

  for (size_t i = 0; i < kMalletCount; ++i)
  {
    if (mallets[i].MIDInote == note)
    {
      scheduleNote(i, noteMillis, velocity);
      return;
    }
  }
}

void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp)
{
  (void)channel;
  (void)note;
  (void)velocity;
  (void)timestamp;
}

} // namespace

void setup()
{
  Serial.begin(115200);
  const unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 2000))
  {
    delay(10);
  }

  EEPROM.begin(kEepromBytes);
  initializeVelocityCalibrationDefaults();
  setupAudioCapture();

  bool loaded = loadCalibrationFromEeprom();
  if (!loaded && kRunSelfCalibrationOnBootWhenMissing)
  {
    runFullCalibration();
    loaded = true;
  }

  if (!loaded)
  {
    for (size_t i = 0; i < kMalletCount; ++i)
    {
      const unsigned long lag = 300;
      mallets[i].setDelay(lag);
      applyMalletTimingFromLag(mallets[i], lag);
    }
  }

  for (size_t i = 0; i < kMalletCount; ++i)
  {
    mallets[i].setRetriggerGap(kMalletRetriggerGapMs);
  }

  BLEMidiServer.begin(kBleDeviceName);
  BLEMidiServer.setOnConnectCallback([]() {
    Serial.println("BLE MIDI connected");
  });
  BLEMidiServer.setOnDisconnectCallback([]() {
    Serial.println("BLE MIDI disconnected");
  });
  BLEMidiServer.setNoteOnCallback(handleNoteOn);
  BLEMidiServer.setNoteOffCallback(handleNoteOff);

  Serial.println("RobotDrum ready");
  Serial.print("Serial commands: status, cal, calvel, setnote <idx> <midi>, audiostatus, micdiag, micprobe [ms], micch left|right, micshift <0..");
  Serial.print(kMicGainShiftMaxBits);
  Serial.println(">, ffttest [index]");
}

void loop()
{
  serviceFftAnalysis();
  updateMallets();
  handleSerialCommands();
}
