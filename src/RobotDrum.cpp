#include <Arduino.h>
#include <BLEMidi.h>
#include <EEPROM.h>
#include <fft.h>
#include "driver/i2s.h"
#include "I2SMEMSSampler.h"
#include "AudioDiagnostics.h"
#include "Calibration.h"
#include "CalibrationRuntime.h"
#include "SerialCommands.h"
#include "mallet.h"

namespace
{
constexpr unsigned long kMidiLeadTimeMs = 1000;
constexpr unsigned long kBleTimestampRolloverMs = 8192;
constexpr unsigned long kCalibrationImpactTimeoutMs = 2500;
constexpr unsigned long kCalibrationMalletIdleTimeoutMs = 4000;
constexpr unsigned long kImpactDetectGuardMs = 60;
constexpr unsigned long kCalibrationHardMaxLagMs = 400;
constexpr unsigned long kCalibrationSoftLagSlackMs = 180;
constexpr unsigned long kCalibrationSoftMaxLagFloorMs = 300;
constexpr unsigned long kRingdownTimeoutMs = 1800;
constexpr unsigned long kRingdownStableMs = 80;
constexpr unsigned long kFftTimeoutMs = 1200;
constexpr unsigned long kMalletRetriggerGapMs = 60;
constexpr unsigned long kPitchCaptureOffsetMs = 600;
constexpr unsigned long kCalibrationInterStrikeDelayMs = 120;
constexpr unsigned long kCalibrationInterMalletDelayMs = 200;
constexpr unsigned long kCalibrationSoftRetryDelayMs = 60;
constexpr int kSampleRateHz = 16384;
constexpr int kAudioBufferSamples = 128;
constexpr int kFftLength = 4096;
constexpr int kCalibrationRepeats = 3;
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
constexpr int32_t kSoftMinRiseHardDivisor = 5;
constexpr uint16_t kSoftPowerInitial = 640;
constexpr uint16_t kSoftPowerStep = 120;
constexpr uint16_t kSoftPowerMax = 900;
constexpr int kSoftPowerAttempts = 4;
constexpr float kFftMinDetectHz = 90.0f;
constexpr float kFftMaxDetectHz = 2200.0f;
constexpr float kFftFundamentalMinHz = 95.0f;
constexpr float kFftFundamentalMaxHz = 900.0f;
constexpr float kFftMinMagnitude = 20.0f;
constexpr float kFftSecondHarmonicWeight = 0.55f;
constexpr float kFftThirdHarmonicWeight = 0.30f;
constexpr float kFftFourthHarmonicWeight = 0.15f;
constexpr float kFftSubharmonicPenaltyWeight = 0.70f;
constexpr float kFftMinFundamentalToHarmonicRatio = 0.35f;
constexpr float kFftWeakFundamentalPenalty = 6.0f;
constexpr float kFftSecondWindowRelativeMagnitudeFloor = 0.60f;
constexpr float kStereoSlotDominanceRatio = 2.0f;
constexpr float kAutocorrMinDetectHz = 95.0f;
constexpr float kAutocorrMaxDetectHz = 900.0f;
constexpr float kAutocorrMinCorrelation = 0.20f;
constexpr float kAutocorrDisagreementRatio = 1.10f;
constexpr float kMeasuredSampleRateMinRatio = 0.96f;
constexpr float kMeasuredSampleRateMaxRatio = 1.04f;
constexpr float kMeasuredSampleRateSmoothing = 0.10f;
constexpr bool kRunSelfCalibrationOnBootWhenMissing = true;
constexpr char kBleDeviceName[] = "MusicRobot";

enum MicChannelSelect
{
  MIC_CHANNEL_LEFT = 0,
  MIC_CHANNEL_RIGHT = 1,
};

enum MicSampleMode
{
  MIC_SAMPLE_MODE_AUTO = 0,
  MIC_SAMPLE_MODE_STEREO_PAIRS = 1,
  MIC_SAMPLE_MODE_MONO_STREAM = 2,
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

struct ScheduledStrike
{
  Mallet::StrikeProfile profile;
  unsigned long lagMs;
};

struct FftWindowEstimate
{
  float frequencyHz;
  float magnitude;
  bool ready;
};

constexpr size_t kEepromBytes = 512;

I2SSampler *gSampler = nullptr;
TaskHandle_t gAudioWriterTaskHandle = nullptr;

volatile int32_t gFilteredMic = 0;
volatile int32_t gDynamicMicPeak = 1;
volatile float gNormalizedMic = 0.0f;
volatile int32_t gMicDc = 0;
volatile int32_t gMicLeftPeak = 0;
volatile int32_t gMicRightPeak = 0;
volatile int gMicChannelSelect = MIC_CHANNEL_LEFT;
volatile int gMicSampleMode = MIC_SAMPLE_MODE_AUTO;
volatile bool gUsingStereoSamplePairs = true;
volatile int gMicGainShiftBits = kMicGainShiftDefaultBits;
volatile bool gCalibrationInProgress = false;

volatile bool gFftCaptureArmed = false;
volatile bool gFftFrameReady = false;
volatile bool gFftFramePending = false;
volatile float gFftDetectedHz = 0.0f;
volatile float gFftDetectedMagnitude = 0.0f;
volatile float gFftAutocorrHz = 0.0f;
volatile float gFftAutocorrCorrelation = 0.0f;
volatile float gFftMeasuredSampleRateHz = static_cast<float>(kSampleRateHz);
volatile float gFftAnalysisSampleRateHz = static_cast<float>(kSampleRateHz);
volatile uint32_t gFftCaptureStartMicros = 0;
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

  return midi;
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

float fftMagnitudeAtBinOrZero(const fft_config_t *plan, int bin, int minBin, int maxBin)
{
  if (bin < minBin || bin > maxBin)
  {
    return 0.0f;
  }
  return fftMagnitudeAtBin(plan, bin);
}

float scoreFundamentalBin(const fft_config_t *plan, int bin, int minBin, int maxBin, float fundamentalMagnitude)
{
  const float secondHarmonic = fftMagnitudeAtBinOrZero(plan, bin * 2, minBin, maxBin);
  const float thirdHarmonic = fftMagnitudeAtBinOrZero(plan, bin * 3, minBin, maxBin);
  const float fourthHarmonic = fftMagnitudeAtBinOrZero(plan, bin * 4, minBin, maxBin);

  float subharmonic = 0.0f;
  const int subharmonicBin = bin / 2;
  if (subharmonicBin >= minBin)
  {
    subharmonic = fftMagnitudeAtBin(plan, subharmonicBin);
  }

  const float eps = 0.001f;
  float score = logf(fundamentalMagnitude + eps) +
                (kFftSecondHarmonicWeight * logf(secondHarmonic + eps)) +
                (kFftThirdHarmonicWeight * logf(thirdHarmonic + eps)) +
                (kFftFourthHarmonicWeight * logf(fourthHarmonic + eps)) -
                (kFftSubharmonicPenaltyWeight * logf(subharmonic + eps));

  const float strongestHarmonic = fmaxf(secondHarmonic, fmaxf(thirdHarmonic, fourthHarmonic));
  if (strongestHarmonic > 1.0f)
  {
    const float ratio = fundamentalMagnitude / strongestHarmonic;
    if (ratio < kFftMinFundamentalToHarmonicRatio)
    {
      score -= kFftWeakFundamentalPenalty;
    }
  }

  return score;
}

float estimateAutocorrelationFrequency(float *samples, int sampleCount, float sampleRateHz, float minHz, float maxHz, float *correlationOut)
{
  if (correlationOut != nullptr)
  {
    *correlationOut = 0.0f;
  }
  if (sampleCount < 32 || minHz <= 0.0f || maxHz <= minHz)
  {
    return 0.0f;
  }

  int minLag = static_cast<int>(floorf(sampleRateHz / maxHz));
  int maxLag = static_cast<int>(ceilf(sampleRateHz / minHz));
  if (minLag < 2)
  {
    minLag = 2;
  }
  if (maxLag >= sampleCount - 4)
  {
    maxLag = sampleCount - 4;
  }
  if (maxLag <= minLag)
  {
    return 0.0f;
  }

  float bestCorr = -1.0f;
  int bestLag = minLag;
  float prevCorr = -1.0f;
  float corrAtBestMinus = -1.0f;
  float corrAtBest = -1.0f;
  float corrAtBestPlus = -1.0f;

  for (int lag = minLag; lag <= maxLag; ++lag)
  {
    float sumXY = 0.0f;
    float sumXX = 0.0f;
    float sumYY = 0.0f;
    const int limit = sampleCount - lag;
    for (int i = 0; i < limit; ++i)
    {
      const float x = samples[i];
      const float y = samples[i + lag];
      sumXY += x * y;
      sumXX += x * x;
      sumYY += y * y;
    }

    float corr = 0.0f;
    const float denom = sqrtf(sumXX * sumYY);
    if (denom > 0.001f)
    {
      corr = sumXY / denom;
    }

    if (corr > bestCorr)
    {
      bestCorr = corr;
      bestLag = lag;
      corrAtBestMinus = prevCorr;
      corrAtBest = corr;
      corrAtBestPlus = -1.0f;
    }
    else if (lag == bestLag + 1)
    {
      corrAtBestPlus = corr;
    }

    prevCorr = corr;
  }

  if (bestCorr < 0.0f)
  {
    return 0.0f;
  }

  float refinedLag = static_cast<float>(bestLag);
  if (corrAtBestMinus >= 0.0f && corrAtBestPlus >= 0.0f)
  {
    const float denom = (corrAtBestMinus - (2.0f * corrAtBest) + corrAtBestPlus);
    if (fabsf(denom) > 0.0001f)
    {
      const float delta = 0.5f * (corrAtBestMinus - corrAtBestPlus) / denom;
      if (delta > -1.0f && delta < 1.0f)
      {
        refinedLag += delta;
      }
    }
  }

  if (correlationOut != nullptr)
  {
    *correlationOut = bestCorr;
  }

  if (refinedLag < 1.0f)
  {
    return 0.0f;
  }
  return sampleRateHz / refinedLag;
}

void executeFftFrame()
{
  float analysisSampleRateHz = gFftAnalysisSampleRateHz;
  if (analysisSampleRateHz < 4000.0f || analysisSampleRateHz > 50000.0f)
  {
    analysisSampleRateHz = static_cast<float>(kSampleRateHz);
  }

  float autocorrCorrelation = 0.0f;
  const float autocorrFrequency = estimateAutocorrelationFrequency(gFftInput, kFftLength, analysisSampleRateHz, kAutocorrMinDetectHz, kAutocorrMaxDetectHz, &autocorrCorrelation);

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

  const int minBin = static_cast<int>(ceilf(kFftMinDetectHz * static_cast<float>(kFftLength) / analysisSampleRateHz));
  const int maxBin = static_cast<int>(floorf(kFftMaxDetectHz * static_cast<float>(kFftLength) / analysisSampleRateHz));
  const int minFundamentalBin = static_cast<int>(ceilf(kFftFundamentalMinHz * static_cast<float>(kFftLength) / analysisSampleRateHz));
  const int maxFundamentalBin = static_cast<int>(floorf(kFftFundamentalMaxHz * static_cast<float>(kFftLength) / analysisSampleRateHz));

  const int nyquistBin = (plan->size / 2) - 1;
  const int searchMinBin = (minBin > 1) ? minBin : 1;
  int searchMaxBin = (maxBin < nyquistBin) ? maxBin : nyquistBin;
  if (searchMaxBin < searchMinBin)
  {
    searchMaxBin = searchMinBin;
  }

  int candidateMinBin = (minFundamentalBin > searchMinBin) ? minFundamentalBin : searchMinBin;
  int candidateMaxBin = (maxFundamentalBin < searchMaxBin) ? maxFundamentalBin : searchMaxBin;
  if (candidateMinBin > candidateMaxBin)
  {
    candidateMinBin = searchMinBin;
    candidateMaxBin = searchMaxBin;
  }

  float bestMagnitude = 0.0f;
  float bestScore = -1000000000.0f;
  int bestBin = candidateMinBin;
  bool foundCandidate = false;

  for (int bin = candidateMinBin; bin <= candidateMaxBin; ++bin)
  {
    const float center = fftMagnitudeAtBin(plan, bin);
    bool localPeak = true;
    if (bin > searchMinBin)
    {
      localPeak = localPeak && (center >= fftMagnitudeAtBin(plan, bin - 1));
    }
    if (bin < searchMaxBin)
    {
      localPeak = localPeak && (center >= fftMagnitudeAtBin(plan, bin + 1));
    }
    if (!localPeak)
    {
      continue;
    }

    const float score = scoreFundamentalBin(plan, bin, searchMinBin, searchMaxBin, center);
    if (!foundCandidate || score > bestScore)
    {
      bestMagnitude = center;
      bestScore = score;
      bestBin = bin;
      foundCandidate = true;
    }
  }

  if (!foundCandidate)
  {
    for (int bin = candidateMinBin; bin <= candidateMaxBin; ++bin)
    {
      const float magnitude = fftMagnitudeAtBin(plan, bin);
      if (magnitude > bestMagnitude)
      {
        bestMagnitude = magnitude;
        bestBin = bin;
      }
    }
  }

  float refinedBin = static_cast<float>(bestBin);
  if (bestBin > searchMinBin && bestBin < searchMaxBin)
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

  const float bestFrequency = refinedBin * analysisSampleRateHz / static_cast<float>(kFftLength);
  gFftAutocorrHz = autocorrFrequency;
  gFftAutocorrCorrelation = autocorrCorrelation;

  float finalFrequency = bestFrequency;
  if (autocorrFrequency > 0.0f && autocorrCorrelation >= kAutocorrMinCorrelation)
  {
    if (bestFrequency <= 0.0f)
    {
      finalFrequency = autocorrFrequency;
    }
    else
    {
      float ratio = bestFrequency / autocorrFrequency;
      if (ratio < 1.0f)
      {
        ratio = 1.0f / ratio;
      }
      if (ratio > kAutocorrDisagreementRatio)
      {
        finalFrequency = autocorrFrequency;
      }
      else
      {
        finalFrequency = 0.5f * (bestFrequency + autocorrFrequency);
      }
    }
  }

  gFftDetectedHz = finalFrequency;
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

bool captureFftWindow(FftWindowEstimate *estimate)
{
  if (estimate != nullptr)
  {
    estimate->frequencyHz = 0.0f;
    estimate->magnitude = 0.0f;
    estimate->ready = false;
  }

  gFftFrameReady = false;
  gFftFramePending = false;
  gFftSampleCursor = 0;
  gFftCaptureStartMicros = micros();
  gFftCaptureArmed = true;

  const unsigned long start = millis();
  while (!gFftFrameReady && (millis() - start < kFftTimeoutMs))
  {
    serviceFftAnalysis();
    updateMallets();
    // Yield so the audio writer task can keep up with incoming I2S buffers.
    delay(1);
  }

  gFftCaptureArmed = false;

  if (!gFftFrameReady)
  {
    return false;
  }

  if (estimate != nullptr)
  {
    estimate->frequencyHz = gFftDetectedHz;
    estimate->magnitude = gFftDetectedMagnitude;
    estimate->ready = true;
  }
  return true;
}

bool estimateIsValid(const FftWindowEstimate &estimate)
{
  return estimate.ready &&
         estimate.frequencyHz > 0.0f &&
         estimate.magnitude >= kFftMinMagnitude;
}

void chooseStableFftEstimate(const FftWindowEstimate &first, const FftWindowEstimate &second, float *frequencyHz, float *magnitude)
{
  auto assignEstimate = [&](const FftWindowEstimate &selected) {
    if (frequencyHz != nullptr)
    {
      *frequencyHz = selected.frequencyHz;
    }
    if (magnitude != nullptr)
    {
      *magnitude = selected.magnitude;
    }
  };

  const bool firstValid = estimateIsValid(first);
  const bool secondValid = estimateIsValid(second);

  if (!firstValid && !secondValid)
  {
    if (frequencyHz != nullptr)
    {
      *frequencyHz = 0.0f;
    }
    if (magnitude != nullptr)
    {
      *magnitude = 0.0f;
    }
    return;
  }

  if (firstValid && !secondValid)
  {
    assignEstimate(first);
    return;
  }
  if (!firstValid && secondValid)
  {
    assignEstimate(second);
    return;
  }

  const int firstMidi = frequencyToMidi(first.frequencyHz);
  const int secondMidi = frequencyToMidi(second.frequencyHz);
  if (firstMidi >= 0 && firstMidi == secondMidi)
  {
    const float firstWeight = (first.magnitude > 1.0f) ? first.magnitude : 1.0f;
    const float secondWeight = (second.magnitude > 1.0f) ? second.magnitude : 1.0f;
    if (frequencyHz != nullptr)
    {
      *frequencyHz = ((first.frequencyHz * firstWeight) + (second.frequencyHz * secondWeight)) / (firstWeight + secondWeight);
    }
    if (magnitude != nullptr)
    {
      *magnitude = (first.magnitude > second.magnitude) ? first.magnitude : second.magnitude;
    }
    return;
  }

  const FftWindowEstimate &lower = (first.frequencyHz <= second.frequencyHz) ? first : second;
  const FftWindowEstimate &higher = (&lower == &first) ? second : first;
  const float harmonicRatio = higher.frequencyHz / lower.frequencyHz;
  if ((harmonicRatio > 1.80f && harmonicRatio < 2.20f) ||
      (harmonicRatio > 2.70f && harmonicRatio < 3.30f))
  {
    assignEstimate(lower);
    return;
  }

  // Prefer the later window when it is still strong enough because
  // the sustained fundamental often outlasts the early harmonic burst.
  if (second.magnitude >= (first.magnitude * kFftSecondWindowRelativeMagnitudeFloor))
  {
    assignEstimate(second);
  }
  else
  {
    assignEstimate(first);
  }
}

bool detectStereoSamplePairs(int32_t *buffer, int32_t sampleCount)
{
  if ((sampleCount % 2) != 0 || sampleCount < 8)
  {
    return false;
  }

  int64_t evenAbsSum = 0;
  int64_t oddAbsSum = 0;
  int64_t pairDiffAbsSum = 0;
  const int pairCount = sampleCount / 2;

  for (int32_t i = 0; i < sampleCount; i += 2)
  {
    const int32_t evenDecoded = decodeMicSample(buffer[i]);
    const int32_t oddDecoded = decodeMicSample(buffer[i + 1]);
    const int32_t evenAbs = (evenDecoded >= 0) ? evenDecoded : -evenDecoded;
    const int32_t oddAbs = (oddDecoded >= 0) ? oddDecoded : -oddDecoded;
    int32_t pairDiff = evenDecoded - oddDecoded;
    if (pairDiff < 0)
    {
      pairDiff = -pairDiff;
    }

    evenAbsSum += evenAbs;
    oddAbsSum += oddAbs;
    pairDiffAbsSum += pairDiff;
  }

  const int64_t smallerAbs = (evenAbsSum < oddAbsSum) ? evenAbsSum : oddAbsSum;
  const int64_t largerAbs = (evenAbsSum > oddAbsSum) ? evenAbsSum : oddAbsSum;
  const float channelEnergyRatio = static_cast<float>(largerAbs + 1) / static_cast<float>(smallerAbs + 1);
  const float meanAbs = static_cast<float>(evenAbsSum + oddAbsSum) / static_cast<float>(sampleCount);
  const float meanPairDiff = static_cast<float>(pairDiffAbsSum) / static_cast<float>(pairCount);
  const float normalizedPairDiff = meanPairDiff / (meanAbs + 1.0f);

  const bool oneSlotMostlySilent = channelEnergyRatio > 6.0f;
  const bool duplicatedStereo = (channelEnergyRatio < 2.0f) && (normalizedPairDiff < 0.015f);
  const bool monoSequential = (channelEnergyRatio < 2.0f) && (normalizedPairDiff > 0.03f);

  if (oneSlotMostlySilent || duplicatedStereo)
  {
    return true;
  }
  if (monoSequential)
  {
    return false;
  }
  return true;
}

void updateMeasuredSampleRate(uint32_t captureStartMicros, uint32_t captureEndMicros)
{
  if (captureStartMicros == 0 || captureEndMicros <= captureStartMicros)
  {
    return;
  }

  const uint32_t elapsedMicros = captureEndMicros - captureStartMicros;
  if (elapsedMicros == 0)
  {
    return;
  }

  const float measuredRate = (static_cast<float>(kFftLength) * 1000000.0f) / static_cast<float>(elapsedMicros);
  if (measuredRate <= 4000.0f || measuredRate >= 50000.0f)
  {
    return;
  }

  gFftMeasuredSampleRateHz = measuredRate;

  const float nominalRate = static_cast<float>(kSampleRateHz);
  const float minAccepted = nominalRate * kMeasuredSampleRateMinRatio;
  const float maxAccepted = nominalRate * kMeasuredSampleRateMaxRatio;
  const float prevRate = gFftAnalysisSampleRateHz;

  if (measuredRate >= minAccepted && measuredRate <= maxAccepted)
  {
    gFftAnalysisSampleRateHz = prevRate + ((measuredRate - prevRate) * kMeasuredSampleRateSmoothing);
  }
  else
  {
    // Keep analysis rate stable when capture timing has a scheduling outlier.
    gFftAnalysisSampleRateHz = prevRate + ((nominalRate - prevRate) * 0.05f);
  }
}

void processAudioBlock(int32_t *buffer, int32_t sampleCount)
{
  bool stereoPairs = ((sampleCount % 2) == 0);
  if (gMicSampleMode == MIC_SAMPLE_MODE_MONO_STREAM)
  {
    stereoPairs = false;
  }
  else if (gMicSampleMode == MIC_SAMPLE_MODE_STEREO_PAIRS)
  {
    stereoPairs = ((sampleCount % 2) == 0);
  }
  else
  {
    stereoPairs = detectStereoSamplePairs(buffer, sampleCount);
  }
  gUsingStereoSamplePairs = stereoPairs;

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

      int32_t selected = (gMicChannelSelect == MIC_CHANNEL_RIGHT) ? rightDecoded : leftDecoded;
      if (gMicSampleMode == MIC_SAMPLE_MODE_AUTO)
      {
        const int32_t largerAbs = (leftAbs > rightAbs) ? leftAbs : rightAbs;
        const int32_t smallerAbs = (leftAbs < rightAbs) ? leftAbs : rightAbs;
        const float dominance = static_cast<float>(largerAbs + 1) / static_cast<float>(smallerAbs + 1);
        if (dominance >= kStereoSlotDominanceRatio)
        {
          selected = (leftAbs >= rightAbs) ? leftDecoded : rightDecoded;
        }
      }
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
          updateMeasuredSampleRate(gFftCaptureStartMicros, micros());
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
        updateMeasuredSampleRate(gFftCaptureStartMicros, micros());
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
  // Keep writer priority above the loop task so we do not starve audio capture
  // during calibration waits on the same core.
  xTaskCreatePinnedToCore(i2sAudioWriterTask, "I2S Writer Task", 8192, gSampler, 2, &gAudioWriterTaskHandle, 1);
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

void applyMalletTimingFromLag(Mallet &mallet, unsigned long lagMs)
{
  CalibrationRuntime::applyMalletTimingFromLag(mallet, lagMs);
}

int detectPitchFromStrike(Mallet &mallet, unsigned long captureDelayMs, float *detectedHz, float *detectedMagnitude)
{
  CalibrationRuntime::waitForMalletIdle(mallet, kCalibrationMalletIdleTimeoutMs);
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

  FftWindowEstimate firstWindow = {};
  FftWindowEstimate secondWindow = {};
  captureFftWindow(&firstWindow);
  captureFftWindow(&secondWindow);

  float frequency = 0.0f;
  float magnitude = 0.0f;
  chooseStableFftEstimate(firstWindow, secondWindow, &frequency, &magnitude);

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
  Calibration::setDefaults(mallets[index]);
}

void initializeVelocityCalibrationDefaults()
{
  Calibration::initializeDefaults(mallets, kMalletCount);
}

ScheduledStrike buildStrikeForVelocity(size_t malletIndex, uint8_t velocity)
{
  Mallet::CalibrationModel model = mallets[malletIndex].getCalibration();
  if (!model.valid)
  {
    setVelocityCalibrationDefaults(malletIndex);
    model = mallets[malletIndex].getCalibration();
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

bool saveCalibrationToEeprom()
{
  return Calibration::saveToEeprom(mallets, kMalletCount, Serial);
}

bool loadCalibrationFromEeprom()
{
  const bool loaded = Calibration::loadFromEeprom(mallets, kMalletCount, Serial);
  if (!loaded)
  {
    return false;
  }

  for (size_t i = 0; i < kMalletCount; ++i)
  {
    const Mallet::CalibrationModel model = mallets[i].getCalibration();
    const unsigned long defaultLag = Calibration::defaultLagFromModel(model);
    mallets[i].setDelay(defaultLag);
    applyMalletTimingFromLag(mallets[i], defaultLag);
  }
  return true;
}

void printMalletStatus()
{
  Calibration::printStatus(mallets, kMalletCount, Serial);
}

void runFftTestForMallet(int index)
{
  CalibrationRuntime::runFftTestForMallet(index);
}

void playCalibrationArpeggio()
{
  CalibrationRuntime::playCalibrationArpeggio();
}

void runVelocityCalibrationOnly()
{
  CalibrationRuntime::runVelocityCalibrationOnly();
}

void runFullCalibration()
{
  CalibrationRuntime::runFullCalibration();
}

void runFullCalibrationForMallet(size_t index)
{
  CalibrationRuntime::runFullCalibrationForMallet(index);
}

void printAudioStatus()
{
  AudioDiagnostics::printAudioStatus();
}

void printMicDiagnostics()
{
  AudioDiagnostics::printMicDiagnostics();
}

void runMicProbe(const String &cmd)
{
  AudioDiagnostics::runMicProbe(cmd);
}

void setMicChannelFromCommand(const String &cmd)
{
  AudioDiagnostics::setMicChannelFromCommand(cmd);
}

void setMicShiftFromCommand(const String &cmd)
{
  AudioDiagnostics::setMicShiftFromCommand(cmd);
}

void setMicModeFromCommand(const String &cmd)
{
  AudioDiagnostics::setMicModeFromCommand(cmd);
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
  const SerialCommands::Handlers handlers = {
    playCalibrationArpeggio,
    runFullCalibration,
    runFullCalibrationForMallet,
    runVelocityCalibrationOnly,
    printMalletStatus,
    printAudioStatus,
    printMicDiagnostics,
    runMicProbe,
    setMicChannelFromCommand,
    setMicShiftFromCommand,
    setMicModeFromCommand,
    runFftTestForMallet,
    setMalletMidiFromCommand,
  };
  SerialCommands::dispatch(command, handlers, kMalletCount, Serial);
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

  CalibrationRuntime::Config calibrationConfig = {};
  calibrationConfig.calibrationImpactTimeoutMs = kCalibrationImpactTimeoutMs;
  calibrationConfig.calibrationMalletIdleTimeoutMs = kCalibrationMalletIdleTimeoutMs;
  calibrationConfig.impactDetectGuardMs = kImpactDetectGuardMs;
  calibrationConfig.calibrationHardMaxLagMs = kCalibrationHardMaxLagMs;
  calibrationConfig.calibrationSoftLagSlackMs = kCalibrationSoftLagSlackMs;
  calibrationConfig.calibrationSoftMaxLagFloorMs = kCalibrationSoftMaxLagFloorMs;
  calibrationConfig.ringdownTimeoutMs = kRingdownTimeoutMs;
  calibrationConfig.ringdownStableMs = kRingdownStableMs;
  calibrationConfig.pitchCaptureOffsetMs = kPitchCaptureOffsetMs;
  calibrationConfig.calibrationInterStrikeDelayMs = kCalibrationInterStrikeDelayMs;
  calibrationConfig.calibrationInterMalletDelayMs = kCalibrationInterMalletDelayMs;
  calibrationConfig.calibrationSoftRetryDelayMs = kCalibrationSoftRetryDelayMs;
  calibrationConfig.calibrationRepeats = kCalibrationRepeats;
  calibrationConfig.impactThresholdHardRise = kImpactThresholdHardRise;
  calibrationConfig.impactThresholdSoftRise = kImpactThresholdSoftRise;
  calibrationConfig.ringdownThresholdRise = kRingdownThresholdRise;
  calibrationConfig.impactThresholdHysteresis = kImpactThresholdHysteresis;
  calibrationConfig.impactNoiseRiseFloor = kImpactNoiseRiseFloor;
  calibrationConfig.impactNoiseMultiplier = kImpactNoiseMultiplier;
  calibrationConfig.hardMinRiseFloor = kHardMinRiseFloor;
  calibrationConfig.softMinRiseFloor = kSoftMinRiseFloor;
  calibrationConfig.softMinRiseHardDivisor = kSoftMinRiseHardDivisor;
  calibrationConfig.softPowerInitial = kSoftPowerInitial;
  calibrationConfig.softPowerStep = kSoftPowerStep;
  calibrationConfig.softPowerMax = kSoftPowerMax;
  calibrationConfig.softPowerAttempts = kSoftPowerAttempts;

  CalibrationRuntime::Callbacks calibrationCallbacks = {};
  calibrationCallbacks.updateMallets = updateMallets;
  calibrationCallbacks.serviceDelay = serviceDelay;
  calibrationCallbacks.detectPitchFromStrike = detectPitchFromStrike;
  calibrationCallbacks.scheduleNote = scheduleNote;
  calibrationCallbacks.saveCalibrationToEeprom = saveCalibrationToEeprom;
  calibrationCallbacks.printMalletStatus = printMalletStatus;
  calibrationCallbacks.prepareMalletsForCalibration = prepareMalletsForCalibration;

  CalibrationRuntime::Dependencies calibrationDeps = {};
  calibrationDeps.mallets = mallets;
  calibrationDeps.malletCount = kMalletCount;
  calibrationDeps.filteredMic = &gFilteredMic;
  calibrationDeps.calibrationInProgress = &gCalibrationInProgress;
  calibrationDeps.log = &Serial;
  calibrationDeps.config = calibrationConfig;
  calibrationDeps.callbacks = calibrationCallbacks;
  CalibrationRuntime::initialize(calibrationDeps);

  AudioDiagnostics::Dependencies audioDeps = {};
  audioDeps.filteredMic = &gFilteredMic;
  audioDeps.dynamicMicPeak = &gDynamicMicPeak;
  audioDeps.normalizedMic = &gNormalizedMic;
  audioDeps.fftCaptureArmed = &gFftCaptureArmed;
  audioDeps.fftFrameReady = &gFftFrameReady;
  audioDeps.fftDetectedHz = &gFftDetectedHz;
  audioDeps.fftDetectedMagnitude = &gFftDetectedMagnitude;
  audioDeps.fftAutocorrHz = &gFftAutocorrHz;
  audioDeps.fftAutocorrCorrelation = &gFftAutocorrCorrelation;
  audioDeps.fftMeasuredSampleRateHz = &gFftMeasuredSampleRateHz;
  audioDeps.fftAnalysisSampleRateHz = &gFftAnalysisSampleRateHz;
  audioDeps.micChannelSelect = &gMicChannelSelect;
  audioDeps.micSampleMode = &gMicSampleMode;
  audioDeps.usingStereoSamplePairs = &gUsingStereoSamplePairs;
  audioDeps.micLeftPeak = &gMicLeftPeak;
  audioDeps.micRightPeak = &gMicRightPeak;
  audioDeps.micDc = &gMicDc;
  audioDeps.micGainShiftBits = &gMicGainShiftBits;
  audioDeps.micChannelLeftValue = MIC_CHANNEL_LEFT;
  audioDeps.micChannelRightValue = MIC_CHANNEL_RIGHT;
  audioDeps.micSampleModeAutoValue = MIC_SAMPLE_MODE_AUTO;
  audioDeps.micSampleModeStereoValue = MIC_SAMPLE_MODE_STEREO_PAIRS;
  audioDeps.micSampleModeMonoValue = MIC_SAMPLE_MODE_MONO_STREAM;
  audioDeps.micGainShiftMaxBits = kMicGainShiftMaxBits;
  audioDeps.micSlotTo24BitShiftBits = kMicSlotTo24BitShiftBits;
  audioDeps.updateMallets = updateMallets;
  audioDeps.log = &Serial;
  AudioDiagnostics::initialize(audioDeps);

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
  Serial.print("Serial commands: status, cal [idx], calvel, arp, setnote <idx> <midi>, audiostatus, micdiag, micprobe [ms], micch left|right, micshift <0..");
  Serial.print(kMicGainShiftMaxBits);
  Serial.println(">, micmode auto|stereo|mono, ffttest [index]");
}

void loop()
{
  serviceFftAnalysis();
  updateMallets();
  handleSerialCommands();
}
