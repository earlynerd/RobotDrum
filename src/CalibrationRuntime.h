#ifndef CALIBRATION_RUNTIME_H
#define CALIBRATION_RUNTIME_H

#include <Arduino.h>
#include "mallet.h"

namespace CalibrationRuntime
{

struct Config
{
  unsigned long calibrationImpactTimeoutMs;
  unsigned long calibrationMalletIdleTimeoutMs;
  unsigned long impactDetectGuardMs;
  unsigned long calibrationHardMaxLagMs;
  unsigned long calibrationSoftLagSlackMs;
  unsigned long calibrationSoftMaxLagFloorMs;
  unsigned long ringdownTimeoutMs;
  unsigned long ringdownStableMs;
  unsigned long pitchCaptureOffsetMs;
  unsigned long calibrationInterStrikeDelayMs;
  unsigned long calibrationInterMalletDelayMs;
  unsigned long calibrationSoftRetryDelayMs;
  int calibrationRepeats;
  int32_t impactThresholdHardRise;
  int32_t impactThresholdSoftRise;
  int32_t ringdownThresholdRise;
  int32_t impactThresholdHysteresis;
  int32_t impactNoiseRiseFloor;
  int32_t impactNoiseMultiplier;
  int32_t hardMinRiseFloor;
  int32_t softMinRiseFloor;
  int32_t softMinRiseHardDivisor;
  uint16_t softPowerInitial;
  uint16_t softPowerStep;
  uint16_t softPowerMax;
  int softPowerAttempts;
};

struct Callbacks
{
  void (*updateMallets)();
  void (*serviceDelay)(unsigned long delayMs);
  int (*detectPitchFromStrike)(Mallet &mallet, unsigned long captureDelayMs, float *detectedHz, float *detectedMagnitude);
  void (*scheduleNote)(size_t malletIndex, unsigned long targetNoteMillis, uint8_t velocity);
  bool (*saveCalibrationToEeprom)();
  void (*printMalletStatus)();
  void (*prepareMalletsForCalibration)();
};

struct Dependencies
{
  Mallet *mallets;
  size_t malletCount;
  volatile int32_t *filteredMic;
  volatile bool *calibrationInProgress;
  Stream *log;
  Config config;
  Callbacks callbacks;
};

void initialize(const Dependencies &deps);
bool waitForMalletIdle(Mallet &mallet, unsigned long timeoutMs);
void applyMalletTimingFromLag(Mallet &mallet, unsigned long lagMs);

void runFftTestForMallet(int index);
void playCalibrationArpeggio();
void runVelocityCalibrationOnly();
void runFullCalibration();
void runFullCalibrationForMallet(size_t index);

} // namespace CalibrationRuntime

#endif
