#include "CalibrationRuntime.h"

#include "Calibration.h"

namespace
{
CalibrationRuntime::Dependencies gDeps = {};
bool gDepsInitialized = false;

constexpr int kMaxCalibrationRepeats = 16;
constexpr unsigned long kFallbackSoftLagMs = 320UL;
constexpr unsigned long kArpeggioTailWaitMs = 900UL;

struct AmbientWindowStats
{
  int32_t peak;
  int32_t minValue;
  int32_t maxValue;
  int32_t peakToPeak;
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

bool dependenciesReady()
{
  return gDepsInitialized &&
         gDeps.mallets != nullptr &&
         gDeps.malletCount > 0 &&
         gDeps.filteredMic != nullptr &&
         gDeps.log != nullptr;
}

int32_t readFilteredMic()
{
  return *gDeps.filteredMic;
}

void updateMalletsTick()
{
  if (gDeps.callbacks.updateMallets != nullptr)
  {
    gDeps.callbacks.updateMallets();
  }
  delay(1);
}

void serviceDelayMs(unsigned long delayMs)
{
  if (gDeps.callbacks.serviceDelay != nullptr)
  {
    gDeps.callbacks.serviceDelay(delayMs);
    return;
  }

  const unsigned long start = millis();
  while (millis() - start < delayMs)
  {
    updateMalletsTick();
  }
}

void setCalibrationInProgress(bool active)
{
  if (gDeps.calibrationInProgress != nullptr)
  {
    *gDeps.calibrationInProgress = active;
  }
}

void configureProbeStrike(Mallet &mallet, int strikePower)
{
  mallet.strikePower = constrain(strikePower, 250, 1023);
  mallet.strikeDuration = 200;
  mallet.coastPower = 0;
  mallet.coastDuration = 1;
  mallet.reboundPower = 0;
  mallet.reboundDuration = 1;
}

void configurePitchProbeStrike(Mallet &mallet, const Mallet::CalibrationModel &model)
{
  const unsigned long lagForTiming = model.valid ? static_cast<unsigned long>(model.hardLagMs) : 140UL;
  CalibrationRuntime::applyMalletTimingFromLag(mallet, lagForTiming);
  mallet.strikePower = constrain(static_cast<int>(model.hardPower), 250, 1023);
}

AmbientWindowStats sampleAmbientWindow(unsigned long windowMs)
{
  AmbientWindowStats stats = {};
  stats.peak = 0;
  stats.minValue = INT32_MAX;
  stats.maxValue = INT32_MIN;

  const unsigned long start = millis();
  while (millis() - start < windowMs)
  {
    const int32_t current = readFilteredMic();
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
    updateMalletsTick();
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
  int32_t runningFloor = readFilteredMic();
  while (millis() - start < timeoutMs)
  {
    const int32_t mic = readFilteredMic();
    if (mic < runningFloor)
    {
      runningFloor = mic;
    }

    const int32_t quietThreshold = runningFloor + gDeps.config.ringdownThresholdRise;
    if (mic <= quietThreshold)
    {
      if (stableStart == 0)
      {
        stableStart = millis();
      }
      if (millis() - stableStart >= gDeps.config.ringdownStableMs)
      {
        return;
      }
    }
    else
    {
      stableStart = 0;
    }

    updateMalletsTick();
  }
}

uint16_t medianLagMs(const unsigned long *values, int count)
{
  if (count <= 0)
  {
    return 0;
  }

  unsigned long sorted[kMaxCalibrationRepeats] = {};
  const int limit = (count > kMaxCalibrationRepeats) ? kMaxCalibrationRepeats : count;
  for (int i = 0; i < limit; ++i)
  {
    sorted[i] = values[i];
  }

  for (int i = 1; i < limit; ++i)
  {
    const unsigned long key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key)
    {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = key;
  }

  if ((limit % 2) == 1)
  {
    return static_cast<uint16_t>(sorted[limit / 2]);
  }
  const unsigned long left = sorted[(limit / 2) - 1];
  const unsigned long right = sorted[limit / 2];
  return static_cast<uint16_t>((left + right) / 2UL);
}

StrikeMeasurement measureStrikeResponse(Mallet &mallet, int32_t thresholdRise, unsigned long maxImpactLagMs, int32_t minRiseRequired)
{
  StrikeMeasurement result = {
    gDeps.config.calibrationImpactTimeoutMs,
    0,
    false,
    readFilteredMic(),
    readFilteredMic() + thresholdRise,
    0,
    minRiseRequired,
  };

  if (!CalibrationRuntime::waitForMalletIdle(mallet, gDeps.config.calibrationMalletIdleTimeoutMs))
  {
    return result;
  }
  waitForRingdown(gDeps.config.ringdownTimeoutMs);
  if (!CalibrationRuntime::waitForMalletIdle(mallet, gDeps.config.calibrationMalletIdleTimeoutMs))
  {
    return result;
  }
  const AmbientWindowStats ambientWindow = sampleAmbientWindow(120);
  int32_t ambient = ambientWindow.peak;
  int32_t adaptiveRise = thresholdRise;
  const int32_t noiseRise = (ambientWindow.peakToPeak * gDeps.config.impactNoiseMultiplier) + gDeps.config.impactNoiseRiseFloor;
  if (noiseRise > adaptiveRise)
  {
    adaptiveRise = noiseRise;
  }
  int32_t triggerThreshold = ambient + adaptiveRise;
  int32_t rearmThreshold = triggerThreshold - gDeps.config.impactThresholdHysteresis;

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
  int32_t guardPeakMic = ambient;
  bool guardPhaseComplete = false;
  const unsigned long startMicros = micros();
  mallet.triggerMallet();
  bool crossedBelowThreshold = (readFilteredMic() <= rearmThreshold);

  while ((micros() - startMicros) < (gDeps.config.calibrationImpactTimeoutMs * 1000UL))
  {
    const int32_t mic = readFilteredMic();
    const unsigned long elapsedMs = (micros() - startMicros) / 1000UL;

    // Track mic peak during guard phase. Some mallets couple motor
    // vibration or EMI into the mic, raising the baseline above the
    // pre-motor ambient. Re-baselining after the guard prevents
    // false-early detection at the guard boundary.
    if (elapsedMs < gDeps.config.impactDetectGuardMs)
    {
      if (mic > guardPeakMic)
      {
        guardPeakMic = mic;
      }
    }
    else if (!guardPhaseComplete)
    {
      guardPhaseComplete = true;
      if (guardPeakMic > ambient)
      {
        ambient = guardPeakMic;
        triggerThreshold = ambient + adaptiveRise;
        rearmThreshold = triggerThreshold - gDeps.config.impactThresholdHysteresis;
        detectWindowPeak = ambient;
        result.ambient = ambient;
        result.triggerThreshold = triggerThreshold;
      }
      crossedBelowThreshold = (mic <= rearmThreshold);
    }

    if (elapsedMs <= maxImpactLagMs && mic > detectWindowPeak)
    {
      detectWindowPeak = mic;
    }

    if (mic <= rearmThreshold)
    {
      crossedBelowThreshold = true;
    }

    if (!result.impactDetected &&
        elapsedMs >= gDeps.config.impactDetectGuardMs &&
        elapsedMs <= maxImpactLagMs &&
        crossedBelowThreshold &&
        mic >= triggerThreshold)
    {
      result.impactDetected = true;
      result.lagMs = elapsedMs;
    }

    updateMalletsTick();
  }

  result.peakRise = (detectWindowPeak > ambient) ? (detectWindowPeak - ambient) : 0;
  if (result.impactDetected && result.peakRise < minRiseRequired)
  {
    result.impactDetected = false;
    result.lagMs = maxImpactLagMs;
  }
  return result;
}

bool calibrateVelocityForMallet(size_t index)
{
  Mallet &mallet = gDeps.mallets[index];
  uint16_t softPower = gDeps.config.softPowerInitial;
  const uint16_t hardPower = 1023;
  int repeats = gDeps.config.calibrationRepeats;
  if (repeats < 1)
  {
    repeats = 1;
  }
  if (repeats > kMaxCalibrationRepeats)
  {
    repeats = kMaxCalibrationRepeats;
  }

  uint32_t softImpactSum = 0;
  uint32_t hardImpactSum = 0;
  uint32_t softPowerSum = 0;
  unsigned long softLagSamples[kMaxCalibrationRepeats] = {};
  unsigned long hardLagSamples[kMaxCalibrationRepeats] = {};
  int softLagCount = 0;
  int hardLagCount = 0;
  int softHits = 0;
  int hardHits = 0;

  for (int r = 0; r < repeats; ++r)
  {
    configureProbeStrike(mallet, hardPower);
    StrikeMeasurement hard = measureStrikeResponse(mallet, gDeps.config.impactThresholdHardRise, gDeps.config.calibrationHardMaxLagMs, gDeps.config.hardMinRiseFloor);
    gDeps.log->print("    hard: ambient=");
    gDeps.log->print(hard.ambient);
    gDeps.log->print(" threshold=");
    gDeps.log->print(hard.triggerThreshold);
    gDeps.log->print(" rise=");
    gDeps.log->print(hard.peakRise);
    gDeps.log->print(" minRise=");
    gDeps.log->print(hard.minRiseRequired);
    gDeps.log->print(" noise=");
    gDeps.log->print(hard.noiseP2P);
    gDeps.log->print(" lag=");
    gDeps.log->print(hard.lagMs);
    gDeps.log->print(" hit=");
    gDeps.log->println(hard.impactDetected ? "yes" : "no");
    if (hard.impactDetected)
    {
      hardImpactSum += static_cast<uint32_t>(hard.peakRise);
      if (hardLagCount < kMaxCalibrationRepeats)
      {
        hardLagSamples[hardLagCount++] = hard.lagMs;
      }
      hardHits++;
    }

    serviceDelayMs(gDeps.config.calibrationInterStrikeDelayMs);
    waitForRingdown(gDeps.config.ringdownTimeoutMs);

    unsigned long softMaxLagMs = gDeps.config.calibrationSoftMaxLagFloorMs;
    int32_t softMinRise = gDeps.config.softMinRiseFloor;
    if (hard.impactDetected)
    {
      softMaxLagMs = hard.lagMs + gDeps.config.calibrationSoftLagSlackMs;
      if (softMaxLagMs < gDeps.config.calibrationSoftMaxLagFloorMs)
      {
        softMaxLagMs = gDeps.config.calibrationSoftMaxLagFloorMs;
      }
      const int32_t derivedSoftMinRise = hard.peakRise / gDeps.config.softMinRiseHardDivisor;
      if (derivedSoftMinRise > softMinRise)
      {
        softMinRise = derivedSoftMinRise;
      }
    }

    StrikeMeasurement soft = {};
    uint16_t selectedSoftPower = softPower;
    bool softAccepted = false;
    for (int attempt = 0; attempt < gDeps.config.softPowerAttempts; ++attempt)
    {
      const uint16_t candidatePower = static_cast<uint16_t>(constrain(static_cast<int>(softPower) + (attempt * static_cast<int>(gDeps.config.softPowerStep)),
                                                                       static_cast<int>(gDeps.config.softPowerInitial),
                                                                       static_cast<int>(gDeps.config.softPowerMax)));
      configureProbeStrike(mallet, candidatePower);
      soft = measureStrikeResponse(mallet, gDeps.config.impactThresholdSoftRise, softMaxLagMs, softMinRise);

      gDeps.log->print("    soft: pwr=");
      gDeps.log->print(candidatePower);
      gDeps.log->print(" ambient=");
      gDeps.log->print(soft.ambient);
      gDeps.log->print(" threshold=");
      gDeps.log->print(soft.triggerThreshold);
      gDeps.log->print(" rise=");
      gDeps.log->print(soft.peakRise);
      gDeps.log->print(" minRise=");
      gDeps.log->print(soft.minRiseRequired);
      gDeps.log->print(" noise=");
      gDeps.log->print(soft.noiseP2P);
      gDeps.log->print(" lag=");
      gDeps.log->print(soft.lagMs);
      gDeps.log->print(" hit=");
      gDeps.log->println(soft.impactDetected ? "yes" : "no");

      if (soft.impactDetected)
      {
        selectedSoftPower = candidatePower;
        softAccepted = true;
        break;
      }

      serviceDelayMs(gDeps.config.calibrationSoftRetryDelayMs);
      waitForRingdown(gDeps.config.ringdownTimeoutMs);
    }

    if (softAccepted)
    {
      softPower = selectedSoftPower;
      softPowerSum += static_cast<uint32_t>(selectedSoftPower);
      softImpactSum += static_cast<uint32_t>(soft.peakRise);
      if (softLagCount < kMaxCalibrationRepeats)
      {
        softLagSamples[softLagCount++] = soft.lagMs;
      }
      softHits++;
    }

    serviceDelayMs(gDeps.config.calibrationInterStrikeDelayMs);
    waitForRingdown(gDeps.config.ringdownTimeoutMs);
  }

  // Preserve existing rebound tuning so cal doesn't erase calrebound results.
  const Mallet::CalibrationModel prev = mallet.getCalibration();

  Mallet::CalibrationModel model = {};
  model.strikePct = prev.strikePct;
  model.reboundPeakPwr = prev.reboundPeakPwr;
  model.softPower = (softHits > 0) ? static_cast<uint16_t>(softPowerSum / softHits) : static_cast<uint16_t>(constrain(static_cast<int>(gDeps.config.softPowerInitial + (2 * gDeps.config.softPowerStep)), 250, static_cast<int>(hardPower)));
  model.hardPower = hardPower;
  model.hardLagMs = (hardLagCount > 0) ? medianLagMs(hardLagSamples, hardLagCount) : 320;
  model.softLagMs = (softLagCount > 0) ? medianLagMs(softLagSamples, softLagCount) : static_cast<uint16_t>(model.hardLagMs + 80);
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
  mallet.setCalibration(model);

  const unsigned long defaultLag = Calibration::defaultLagFromModel(model);
  mallet.setDelay(defaultLag);
  CalibrationRuntime::applyMalletTimingFromLag(mallet, defaultLag);

  gDeps.log->print("  velocity: softLag=");
  gDeps.log->print(model.softLagMs);
  gDeps.log->print(" hardLag=");
  gDeps.log->print(model.hardLagMs);
  gDeps.log->print(" softImpact=");
  gDeps.log->print(model.softImpact);
  gDeps.log->print(" hardImpact=");
  gDeps.log->print(model.hardImpact);
  gDeps.log->print(" valid=");
  gDeps.log->println(model.valid ? "yes" : "no");

  return model.valid;
}

void runFullCalibrationStepForMallet(size_t index)
{
  gDeps.mallets[index].abortAndClearQueue();
  gDeps.log->print("Calibrating mallet #");
  gDeps.log->println(index);

  calibrateVelocityForMallet(index);

  const Mallet::CalibrationModel model = gDeps.mallets[index].getCalibration();
  configurePitchProbeStrike(gDeps.mallets[index], model);
  waitForRingdown(gDeps.config.ringdownTimeoutMs);
  serviceDelayMs(gDeps.config.calibrationInterStrikeDelayMs);

  float detectedHz = 0.0f;
  float detectedMagnitude = 0.0f;
  const unsigned long captureDelayMs = static_cast<unsigned long>(model.hardLagMs + gDeps.config.pitchCaptureOffsetMs);
  int detectedPitch = -1;
  if (gDeps.callbacks.detectPitchFromStrike != nullptr)
  {
    detectedPitch = gDeps.callbacks.detectPitchFromStrike(gDeps.mallets[index], captureDelayMs, &detectedHz, &detectedMagnitude);
  }
  if (detectedPitch >= 0)
  {
    gDeps.mallets[index].setMidiPitch(detectedPitch);
  }

  gDeps.log->print("  pitchHz=");
  gDeps.log->print(detectedHz);
  gDeps.log->print(" mag=");
  gDeps.log->print(detectedMagnitude);
  gDeps.log->print(" detected=");
  gDeps.log->print(detectedPitch);
  gDeps.log->print(" midi=");
  gDeps.log->println(gDeps.mallets[index].getMidiPitch());

  waitForRingdown(gDeps.config.ringdownTimeoutMs);
  serviceDelayMs(gDeps.config.calibrationInterMalletDelayMs);
}

// --- Rebound auto-tuning ---
//
// Sweeps coast timing and rebound brake power to find the profile that
// produces the quietest settle after a strike. Intended for use with a
// non-resonant dummy drum so the mic hears only mechanical noise.
//
// Strategy: two sequential 1D sweeps (coast then power) rather than a
// full 2D grid, keeping the trial count manageable (~14 per mallet).

constexpr int kReboundSettleWindowMs = 1200;
constexpr int kReboundInterTrialDelayMs = 150;
constexpr int kReboundStrikeSteps = 7;
constexpr int kReboundPowerSteps = 8;
// Strike duration as % of lag. The rubber ball bounce means the mallet
// leaves the drum almost immediately, so shorter values release sooner
// and let braking start earlier.
constexpr uint8_t kReboundStrikeValues[kReboundStrikeSteps] = {70, 90, 110, 130, 160, 200, 250};
// Peak rebound brake PWM — wider range since elastic bounce preserves
// much of the strike energy. Heavier mallets may need 600+ to brake.
constexpr uint16_t kReboundPowerValues[kReboundPowerSteps] = {0, 50, 100, 170, 250, 400, 600, 800};
constexpr uint8_t kReboundDefaultStrikePct = 100;
constexpr uint16_t kReboundDefaultPower = 250;

unsigned long totalProfileDuration(const Mallet::StrikeProfile &profile)
{
  unsigned long total = 0;
  for (uint8_t i = 0; i < profile.count; ++i)
  {
    total += profile.segments[i].durationMs;
  }
  return total;
}

// Build a shaped test profile with specific strike duration and rebound power.
// strikePct = strike duration as % of lag (100 = cut power right at contact).
// No coast phase — the elastic bounce means the mallet is already moving
// away from the drum by the time the motor releases.
Mallet::StrikeProfile buildReboundTestProfile(
    const Mallet::CalibrationModel &model,
    uint8_t strikePct,
    uint16_t reboundPwr)
{
  const int strikePwr = constrain(static_cast<int>(model.hardPower), 250, 1023);
  const float lagF = static_cast<float>(model.hardLagMs);

  const int totalStrikeDur = constrain(static_cast<int>(lagF * static_cast<float>(strikePct) / 100.0f), 30, 600);
  const int totalReboundDur = constrain(static_cast<int>(lagF * 2.5f), 200, 600);
  const int clampedRebPwr = constrain(static_cast<int>(reboundPwr), 0, 1023);

  Mallet::StrikeProfile prof = {};
  prof.count = 0;

  // Strike ramp + hold
  constexpr int kRampStepMs = 5;
  if (totalStrikeDur > 20)
  {
    prof.segments[prof.count++] = {
      static_cast<uint16_t>(strikePwr / 3),
      static_cast<uint16_t>(kRampStepMs)
    };
    prof.segments[prof.count++] = {
      static_cast<uint16_t>((strikePwr * 2) / 3),
      static_cast<uint16_t>(kRampStepMs)
    };
    prof.segments[prof.count++] = {
      static_cast<uint16_t>(strikePwr),
      static_cast<uint16_t>(totalStrikeDur - (2 * kRampStepMs))
    };
  }
  else
  {
    prof.segments[prof.count++] = {
      static_cast<uint16_t>(strikePwr),
      static_cast<uint16_t>(totalStrikeDur)
    };
  }

  // No coast — go straight to rebound brake.
  // Tapered brake: 100% / 40% / 12.5% of peak power.
  if (totalReboundDur > 6 && prof.count + 3 <= Mallet::kMaxStrikeSegments)
  {
    const int seg1 = (totalReboundDur * 2) / 5;
    const int seg2 = (totalReboundDur * 3) / 10;
    const int seg3 = totalReboundDur - seg1 - seg2;
    prof.segments[prof.count++] = {
      static_cast<uint16_t>(clampedRebPwr),
      static_cast<uint16_t>(seg1)
    };
    prof.segments[prof.count++] = {
      static_cast<uint16_t>((clampedRebPwr * 2) / 5),
      static_cast<uint16_t>(seg2)
    };
    prof.segments[prof.count++] = {
      static_cast<uint16_t>(clampedRebPwr / 8),
      static_cast<uint16_t>(seg3)
    };
  }
  else if (totalReboundDur > 0)
  {
    prof.segments[prof.count++] = {
      static_cast<uint16_t>(clampedRebPwr),
      static_cast<uint16_t>(totalReboundDur)
    };
  }

  return prof;
}

// Strike with a candidate profile, then measure the total acoustic energy
// in a window after the profile completes. Lower energy = quieter settle.
//
// Streams timestamped mic samples to serial for offline analysis.
// Output format (CSV):
//   R,<strikePct>,<reboundPwr>,<phase>,<tMs>,<mic>,<baseline>
//   phase: S=strike/rebound profile active, T=settle measurement window
//
// Downsample factor controls how often samples are emitted (every Nth ms)
// to avoid flooding serial at 115200 baud.
constexpr int kStreamDownsampleMs = 4;

struct TrialResult
{
  uint32_t settleEnergy;  // total |mic - baseline| during settle window
  int32_t peakImpact;     // peak mic reading during strike phase (above baseline)
};

TrialResult measureTrial(
    Mallet &mallet,
    const Mallet::StrikeProfile &profile,
    uint8_t strikePct,
    uint16_t reboundPwr)
{
  TrialResult result = {UINT32_MAX, 0};

  if (!CalibrationRuntime::waitForMalletIdle(mallet, gDeps.config.calibrationMalletIdleTimeoutMs))
  {
    return result;
  }
  waitForRingdown(gDeps.config.ringdownTimeoutMs);

  // Sample ambient baseline
  const AmbientWindowStats ambient = sampleAmbientWindow(80);
  const int32_t baseline = (ambient.minValue + ambient.maxValue) / 2;

  // Fire the test profile
  const unsigned long strikeStart = millis();
  mallet.delayedTrigger(0, profile);

  // Stream mic during strike+rebound phase, track peak impact
  const unsigned long profileMs = totalProfileDuration(profile);
  unsigned long lastStreamMs = 0;
  int32_t peakMic = baseline;
  while (millis() - strikeStart < profileMs + 10)
  {
    const int32_t mic = readFilteredMic();
    if (mic > peakMic)
    {
      peakMic = mic;
    }

    const unsigned long elapsed = millis() - strikeStart;
    if (elapsed - lastStreamMs >= static_cast<unsigned long>(kStreamDownsampleMs))
    {
      lastStreamMs = elapsed;
      gDeps.log->print("R,");
      gDeps.log->print(strikePct);
      gDeps.log->print(",");
      gDeps.log->print(reboundPwr);
      gDeps.log->print(",S,");
      gDeps.log->print(elapsed);
      gDeps.log->print(",");
      gDeps.log->print(mic);
      gDeps.log->print(",");
      gDeps.log->println(baseline);
    }
    updateMalletsTick();
  }

  result.peakImpact = (peakMic > baseline) ? (peakMic - baseline) : 0;

  // Measure and stream the settle window
  uint64_t energy = 0;
  const unsigned long settleStart = millis();
  lastStreamMs = 0;
  while (millis() - settleStart < static_cast<unsigned long>(kReboundSettleWindowMs))
  {
    const int32_t mic = readFilteredMic();
    const int32_t delta = mic - baseline;
    energy += static_cast<uint64_t>(abs(delta));

    const unsigned long elapsed = millis() - settleStart;
    if (elapsed - lastStreamMs >= static_cast<unsigned long>(kStreamDownsampleMs))
    {
      lastStreamMs = elapsed;
      gDeps.log->print("R,");
      gDeps.log->print(strikePct);
      gDeps.log->print(",");
      gDeps.log->print(reboundPwr);
      gDeps.log->print(",T,");
      gDeps.log->print(elapsed);
      gDeps.log->print(",");
      gDeps.log->print(mic);
      gDeps.log->print(",");
      gDeps.log->println(baseline);
    }
    updateMalletsTick();
  }

  result.settleEnergy = (energy > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(energy);
  return result;
}

// Run the two-pass sweep for one mallet. Returns true if tuning succeeded.
//
// Pass 1: Sweep strike duration — choose the duration that produces the
//   LOUDEST hit (highest peak mic). Strike duration controls how long the
//   motor drives before releasing. Too short = mallet hasn't reached full
//   velocity at contact. Too long = motor fights the rebound unnecessarily.
//
// Pass 2: Sweep rebound brake power at the best strike duration — choose
//   the power that produces the QUIETEST settle (lowest acoustic energy
//   in the window after the profile completes).
bool tuneReboundForMallet(size_t index)
{
  Mallet &mallet = gDeps.mallets[index];
  Mallet::CalibrationModel model = mallet.getCalibration();
  if (!model.valid)
  {
    gDeps.log->println("  skipping: no valid calibration");
    return false;
  }

  gDeps.log->println("  pass 1: sweep strike duration (maximize impact)");

  // Pass 1: sweep strike% with default rebound power, pick highest peak
  uint8_t bestStrike = kReboundDefaultStrikePct;
  int32_t bestStrikePeak = 0;

  for (int i = 0; i < kReboundStrikeSteps; ++i)
  {
    const uint8_t strikePct = kReboundStrikeValues[i];
    const Mallet::StrikeProfile prof = buildReboundTestProfile(model, strikePct, kReboundDefaultPower);
    const TrialResult trial = measureTrial(mallet, prof, strikePct, kReboundDefaultPower);

    gDeps.log->print("    strike=");
    gDeps.log->print(strikePct);
    gDeps.log->print("% peak=");
    gDeps.log->print(trial.peakImpact);
    gDeps.log->print(" energy=");
    gDeps.log->println(trial.settleEnergy);

    if (trial.peakImpact > bestStrikePeak)
    {
      bestStrikePeak = trial.peakImpact;
      bestStrike = strikePct;
    }

    serviceDelayMs(kReboundInterTrialDelayMs);
  }

  gDeps.log->print("  best strike=");
  gDeps.log->print(bestStrike);
  gDeps.log->print("% peak=");
  gDeps.log->println(bestStrikePeak);

  gDeps.log->println("  pass 2: sweep rebound power (minimize settle)");

  // Pass 2: sweep rebound power with the best strike duration, pick lowest energy
  uint16_t bestPower = kReboundDefaultPower;
  uint32_t bestPowerEnergy = UINT32_MAX;

  for (int i = 0; i < kReboundPowerSteps; ++i)
  {
    const uint16_t rebPwr = kReboundPowerValues[i];
    const Mallet::StrikeProfile prof = buildReboundTestProfile(model, bestStrike, rebPwr);
    const TrialResult trial = measureTrial(mallet, prof, bestStrike, rebPwr);

    gDeps.log->print("    pwr=");
    gDeps.log->print(rebPwr);
    gDeps.log->print(" peak=");
    gDeps.log->print(trial.peakImpact);
    gDeps.log->print(" energy=");
    gDeps.log->println(trial.settleEnergy);

    if (trial.settleEnergy < bestPowerEnergy)
    {
      bestPowerEnergy = trial.settleEnergy;
      bestPower = rebPwr;
    }

    serviceDelayMs(kReboundInterTrialDelayMs);
  }

  gDeps.log->print("  best: strike=");
  gDeps.log->print(bestStrike);
  gDeps.log->print("% pwr=");
  gDeps.log->println(bestPower);

  model.strikePct = bestStrike;
  model.reboundPeakPwr = bestPower;
  mallet.setCalibration(model);

  return true;
}

} // namespace

namespace CalibrationRuntime
{

void initialize(const Dependencies &deps)
{
  gDeps = deps;
  gDepsInitialized = true;
}

bool waitForMalletIdle(Mallet &mallet, unsigned long timeoutMs)
{
  const unsigned long start = millis();
  while (!mallet.isIdle() && (millis() - start < timeoutMs))
  {
    updateMalletsTick();
  }
  return mallet.isIdle();
}

void applyMalletTimingFromLag(Mallet &mallet, unsigned long lagMs)
{
  const unsigned long strikeDuration = constrain(static_cast<unsigned long>(lagMs * 1.3f), 100UL, 1600UL);
  const unsigned long coastDuration = constrain(static_cast<unsigned long>(lagMs * 0.10f), 0UL, 300UL);
  const unsigned long reboundDuration = constrain(static_cast<unsigned long>(lagMs * 1.0f), 120UL, 2000UL);

  mallet.strikePower = 1023;
  mallet.strikeDuration = static_cast<int>(strikeDuration);
  mallet.coastPower = 0;
  mallet.coastDuration = static_cast<int>(coastDuration);
  mallet.reboundPower = 100;
  mallet.reboundDuration = static_cast<int>(reboundDuration);
}

void runFftTestForMallet(int index)
{
  if (!dependenciesReady())
  {
    return;
  }

  if (index < 0 || index >= static_cast<int>(gDeps.malletCount))
  {
    gDeps.log->print("ffttest mallet index out of range 0..");
    gDeps.log->println(static_cast<int>(gDeps.malletCount) - 1);
    return;
  }

  const size_t malletIndex = static_cast<size_t>(index);
  Mallet &mallet = gDeps.mallets[malletIndex];
  const Mallet::CalibrationModel model = mallet.getCalibration();
  configurePitchProbeStrike(mallet, model);
  waitForRingdown(gDeps.config.ringdownTimeoutMs);
  serviceDelayMs(gDeps.config.calibrationInterStrikeDelayMs);

  float detectedHz = 0.0f;
  float detectedMagnitude = 0.0f;
  unsigned long captureDelayMs = 120;
  if (model.valid)
  {
    captureDelayMs = static_cast<unsigned long>(model.hardLagMs + gDeps.config.pitchCaptureOffsetMs);
  }

  int detectedPitch = -1;
  if (gDeps.callbacks.detectPitchFromStrike != nullptr)
  {
    detectedPitch = gDeps.callbacks.detectPitchFromStrike(mallet, captureDelayMs, &detectedHz, &detectedMagnitude);
  }

  gDeps.log->print("ffttest mallet=");
  gDeps.log->print(index);
  gDeps.log->print(" hz=");
  gDeps.log->print(detectedHz, 2);
  gDeps.log->print(" mag=");
  gDeps.log->print(detectedMagnitude, 2);
  gDeps.log->print(" midi=");
  gDeps.log->println(detectedPitch);
}

void playCalibrationArpeggio()
{
  if (!dependenciesReady())
  {
    return;
  }

  gDeps.log->println("Calibration arpeggio (lag-aware)");
  size_t order[16] = {};
  if (gDeps.malletCount > 16)
  {
    gDeps.log->println("Arpeggio skipped: mallet count exceeds internal limit");
    return;
  }

  for (size_t i = 0; i < gDeps.malletCount; ++i)
  {
    order[i] = i;
  }

  for (size_t i = 0; i + 1 < gDeps.malletCount; ++i)
  {
    size_t best = i;
    for (size_t j = i + 1; j < gDeps.malletCount; ++j)
    {
      if (gDeps.mallets[order[j]].getMidiPitch() < gDeps.mallets[order[best]].getMidiPitch())
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

  for (size_t i = 0; i < gDeps.malletCount; ++i)
  {
    waitForMalletIdle(gDeps.mallets[i], gDeps.config.calibrationMalletIdleTimeoutMs);
  }

  size_t sequence[(16 * 2) - 1] = {};
  size_t sequenceCount = 0;
  for (size_t i = 0; i < gDeps.malletCount; ++i)
  {
    sequence[sequenceCount++] = order[i];
  }
  for (int i = static_cast<int>(gDeps.malletCount) - 2; i >= 0; --i)
  {
    sequence[sequenceCount++] = order[static_cast<size_t>(i)];
  }

  constexpr uint8_t kArpeggioVelocity = 1;
  const unsigned long now = millis();
  const unsigned long firstNoteMillis = now + 120;
  unsigned long maxLagMs = 0;

  for (size_t step = 0; step < sequenceCount; ++step)
  {
    const size_t malletIndex = sequence[step];
    const unsigned long targetNoteMillis = firstNoteMillis + (step * gDeps.config.calibrationInterMalletDelayMs);
    if (gDeps.callbacks.scheduleNote != nullptr)
    {
      gDeps.callbacks.scheduleNote(malletIndex, targetNoteMillis, kArpeggioVelocity);
    }

    const Mallet::CalibrationModel model = gDeps.mallets[malletIndex].getCalibration();
    const unsigned long lagMs = model.valid
                                  ? static_cast<unsigned long>(model.softLagMs)
                                  : kFallbackSoftLagMs;
    if (lagMs > maxLagMs)
    {
      maxLagMs = lagMs;
    }

    gDeps.log->print("  arp #");
    gDeps.log->print(malletIndex);
    gDeps.log->print(" midi=");
    gDeps.log->print(gDeps.mallets[malletIndex].getMidiPitch());
    gDeps.log->print(" noteAt=");
    gDeps.log->print(targetNoteMillis);
    gDeps.log->print(" lag=");
    gDeps.log->println(lagMs);
  }

  const unsigned long arpeggioSpanMs = (sequenceCount > 0) ? ((sequenceCount - 1) * gDeps.config.calibrationInterMalletDelayMs) : 0;
  const unsigned long tailWaitMs = maxLagMs + kArpeggioTailWaitMs;
  serviceDelayMs(arpeggioSpanMs + tailWaitMs);
}

void runVelocityCalibrationOnly()
{
  if (!dependenciesReady())
  {
    return;
  }

  setCalibrationInProgress(true);
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  gDeps.log->println("Starting velocity calibration");
  for (size_t i = 0; i < gDeps.malletCount; ++i)
  {
    gDeps.mallets[i].abortAndClearQueue();
    gDeps.log->print("Calibrating velocity for mallet #");
    gDeps.log->println(i);
    calibrateVelocityForMallet(i);
    waitForRingdown(gDeps.config.ringdownTimeoutMs);
    serviceDelayMs(gDeps.config.calibrationInterMalletDelayMs);
  }
  if (gDeps.callbacks.saveCalibrationToEeprom != nullptr)
  {
    gDeps.callbacks.saveCalibrationToEeprom();
  }
  if (gDeps.callbacks.printMalletStatus != nullptr)
  {
    gDeps.callbacks.printMalletStatus();
  }
  gDeps.log->println("Velocity calibration complete");
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  setCalibrationInProgress(false);
}

void runFullCalibration()
{
  if (!dependenciesReady())
  {
    return;
  }

  setCalibrationInProgress(true);
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  gDeps.log->println("Starting full calibration");

  for (size_t i = 0; i < gDeps.malletCount; ++i)
  {
    runFullCalibrationStepForMallet(i);
  }

  playCalibrationArpeggio();
  if (gDeps.callbacks.saveCalibrationToEeprom != nullptr)
  {
    gDeps.callbacks.saveCalibrationToEeprom();
  }
  if (gDeps.callbacks.printMalletStatus != nullptr)
  {
    gDeps.callbacks.printMalletStatus();
  }
  gDeps.log->println("Full calibration complete");
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  setCalibrationInProgress(false);
}

void runFullCalibrationForMallet(size_t index)
{
  if (!dependenciesReady())
  {
    return;
  }
  if (index >= gDeps.malletCount)
  {
    gDeps.log->print("cal mallet index out of range 0..");
    gDeps.log->println(static_cast<int>(gDeps.malletCount) - 1);
    return;
  }

  setCalibrationInProgress(true);
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  gDeps.log->print("Starting full calibration for mallet #");
  gDeps.log->println(index);

  runFullCalibrationStepForMallet(index);

  if (gDeps.callbacks.saveCalibrationToEeprom != nullptr)
  {
    gDeps.callbacks.saveCalibrationToEeprom();
  }
  if (gDeps.callbacks.printMalletStatus != nullptr)
  {
    gDeps.callbacks.printMalletStatus();
  }
  gDeps.log->print("Full calibration complete for mallet #");
  gDeps.log->println(index);
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  setCalibrationInProgress(false);
}

void runReboundCalibration()
{
  if (!dependenciesReady())
  {
    return;
  }

  setCalibrationInProgress(true);
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  gDeps.log->println("Starting rebound calibration");
  gDeps.log->println("(use non-resonant dummy drum for best results)");

  for (size_t i = 0; i < gDeps.malletCount; ++i)
  {
    gDeps.mallets[i].abortAndClearQueue();
    gDeps.log->print("Tuning rebound for mallet #");
    gDeps.log->println(i);
    tuneReboundForMallet(i);
    waitForRingdown(gDeps.config.ringdownTimeoutMs);
    serviceDelayMs(gDeps.config.calibrationInterMalletDelayMs);
  }

  if (gDeps.callbacks.saveCalibrationToEeprom != nullptr)
  {
    gDeps.callbacks.saveCalibrationToEeprom();
  }
  if (gDeps.callbacks.printMalletStatus != nullptr)
  {
    gDeps.callbacks.printMalletStatus();
  }
  gDeps.log->println("Rebound calibration complete");
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  setCalibrationInProgress(false);
}

void runReboundCalibrationForMallet(size_t index)
{
  if (!dependenciesReady())
  {
    return;
  }
  if (index >= gDeps.malletCount)
  {
    gDeps.log->print("calrebound mallet index out of range 0..");
    gDeps.log->println(static_cast<int>(gDeps.malletCount) - 1);
    return;
  }

  setCalibrationInProgress(true);
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  gDeps.mallets[index].abortAndClearQueue();
  gDeps.log->print("Tuning rebound for mallet #");
  gDeps.log->println(index);

  tuneReboundForMallet(index);

  if (gDeps.callbacks.saveCalibrationToEeprom != nullptr)
  {
    gDeps.callbacks.saveCalibrationToEeprom();
  }
  if (gDeps.callbacks.printMalletStatus != nullptr)
  {
    gDeps.callbacks.printMalletStatus();
  }
  gDeps.log->print("Rebound calibration complete for mallet #");
  gDeps.log->println(index);
  if (gDeps.callbacks.prepareMalletsForCalibration != nullptr)
  {
    gDeps.callbacks.prepareMalletsForCalibration();
  }
  setCalibrationInProgress(false);
}

} // namespace CalibrationRuntime
