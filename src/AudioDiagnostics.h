#ifndef AUDIO_DIAGNOSTICS_H
#define AUDIO_DIAGNOSTICS_H

#include <Arduino.h>

namespace AudioDiagnostics
{

struct Dependencies
{
  volatile int32_t *filteredMic;
  volatile int32_t *dynamicMicPeak;
  volatile float *normalizedMic;
  volatile bool *fftCaptureArmed;
  volatile bool *fftFrameReady;
  volatile float *fftDetectedHz;
  volatile float *fftDetectedMagnitude;
  volatile float *fftAutocorrHz;
  volatile float *fftAutocorrCorrelation;
  volatile float *fftMeasuredSampleRateHz;
  volatile float *fftAnalysisSampleRateHz;
  volatile int *micChannelSelect;
  volatile int *micSampleMode;
  volatile bool *usingStereoSamplePairs;
  volatile int32_t *micLeftPeak;
  volatile int32_t *micRightPeak;
  volatile int32_t *micDc;
  volatile int *micGainShiftBits;
  int micChannelLeftValue;
  int micChannelRightValue;
  int micSampleModeAutoValue;
  int micSampleModeStereoValue;
  int micSampleModeMonoValue;
  int micGainShiftMaxBits;
  int micSlotTo24BitShiftBits;
  void (*updateMallets)();
  Stream *log;
};

void initialize(const Dependencies &deps);
const char *micSampleModeToString(int mode);
void printAudioStatus();
void printMicDiagnostics();
void runMicProbe(const String &cmd);
void setMicChannelFromCommand(const String &cmd);
void setMicShiftFromCommand(const String &cmd);
void setMicModeFromCommand(const String &cmd);

} // namespace AudioDiagnostics

#endif
