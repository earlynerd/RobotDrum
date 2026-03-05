#include "AudioDiagnostics.h"

namespace
{
AudioDiagnostics::Dependencies gDeps = {};
bool gDepsInitialized = false;

bool dependenciesReady()
{
  return gDepsInitialized &&
         gDeps.log != nullptr &&
         gDeps.filteredMic != nullptr &&
         gDeps.dynamicMicPeak != nullptr &&
         gDeps.normalizedMic != nullptr &&
         gDeps.fftCaptureArmed != nullptr &&
         gDeps.fftFrameReady != nullptr &&
         gDeps.fftDetectedHz != nullptr &&
         gDeps.fftDetectedMagnitude != nullptr &&
         gDeps.fftAutocorrHz != nullptr &&
         gDeps.fftAutocorrCorrelation != nullptr &&
         gDeps.fftMeasuredSampleRateHz != nullptr &&
         gDeps.fftAnalysisSampleRateHz != nullptr &&
         gDeps.micChannelSelect != nullptr &&
         gDeps.micSampleMode != nullptr &&
         gDeps.usingStereoSamplePairs != nullptr &&
         gDeps.micLeftPeak != nullptr &&
         gDeps.micRightPeak != nullptr &&
         gDeps.micDc != nullptr &&
         gDeps.micGainShiftBits != nullptr;
}

void updateMalletsTick()
{
  if (gDeps.updateMallets != nullptr)
  {
    gDeps.updateMallets();
  }
  delay(1);
}

} // namespace

namespace AudioDiagnostics
{

void initialize(const Dependencies &deps)
{
  gDeps = deps;
  gDepsInitialized = true;
}

const char *micSampleModeToString(int mode)
{
  if (mode == gDeps.micSampleModeStereoValue)
  {
    return "stereo";
  }
  if (mode == gDeps.micSampleModeMonoValue)
  {
    return "mono";
  }
  return "auto";
}

void printAudioStatus()
{
  if (!dependenciesReady())
  {
    return;
  }

  gDeps.log->print("Audio filtered=");
  gDeps.log->print(*gDeps.filteredMic);
  gDeps.log->print(" peak=");
  gDeps.log->print(*gDeps.dynamicMicPeak);
  gDeps.log->print(" norm=");
  gDeps.log->print(*gDeps.normalizedMic, 3);
  gDeps.log->print(" fftArmed=");
  gDeps.log->print(*gDeps.fftCaptureArmed ? "yes" : "no");
  gDeps.log->print(" fftReady=");
  gDeps.log->print(*gDeps.fftFrameReady ? "yes" : "no");
  gDeps.log->print(" fftHz=");
  gDeps.log->print(*gDeps.fftDetectedHz, 2);
  gDeps.log->print(" fftMag=");
  gDeps.log->print(*gDeps.fftDetectedMagnitude, 2);
  gDeps.log->print(" acHz=");
  gDeps.log->print(*gDeps.fftAutocorrHz, 2);
  gDeps.log->print(" acCorr=");
  gDeps.log->print(*gDeps.fftAutocorrCorrelation, 3);
  gDeps.log->print(" fsHz=");
  gDeps.log->print(*gDeps.fftMeasuredSampleRateHz, 2);
  gDeps.log->print(" fsUsed=");
  gDeps.log->print(*gDeps.fftAnalysisSampleRateHz, 2);
  gDeps.log->print(" micCh=");
  gDeps.log->print((*gDeps.micChannelSelect == gDeps.micChannelRightValue) ? "right" : "left");
  gDeps.log->print(" micMode=");
  gDeps.log->print(micSampleModeToString(*gDeps.micSampleMode));
  gDeps.log->print(" sampleLayout=");
  gDeps.log->print(*gDeps.usingStereoSamplePairs ? "stereo" : "mono");
  gDeps.log->print(" leftPk=");
  gDeps.log->print(*gDeps.micLeftPeak);
  gDeps.log->print(" rightPk=");
  gDeps.log->print(*gDeps.micRightPeak);
  gDeps.log->print(" dc=");
  gDeps.log->println(*gDeps.micDc);
}

void printMicDiagnostics()
{
  if (!dependenciesReady())
  {
    return;
  }

  gDeps.log->print("micdiag channel=");
  gDeps.log->print((*gDeps.micChannelSelect == gDeps.micChannelRightValue) ? "right" : "left");
  gDeps.log->print(" gainShift=");
  gDeps.log->print(*gDeps.micGainShiftBits);
  gDeps.log->print(" mode=");
  gDeps.log->print(micSampleModeToString(*gDeps.micSampleMode));
  gDeps.log->print(" layout=");
  gDeps.log->print(*gDeps.usingStereoSamplePairs ? "stereo" : "mono");
  gDeps.log->print(" decodeShift=");
  gDeps.log->print(gDeps.micSlotTo24BitShiftBits);
  gDeps.log->print(" leftPeak=");
  gDeps.log->print(*gDeps.micLeftPeak);
  gDeps.log->print(" rightPeak=");
  gDeps.log->print(*gDeps.micRightPeak);
  gDeps.log->print(" filtered=");
  gDeps.log->print(*gDeps.filteredMic);
  gDeps.log->print(" dc=");
  gDeps.log->println(*gDeps.micDc);
}

void runMicProbe(const String &cmd)
{
  if (!dependenciesReady())
  {
    return;
  }

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

  *gDeps.micLeftPeak = 0;
  *gDeps.micRightPeak = 0;

  int32_t filteredMin = INT32_MAX;
  int32_t filteredMax = INT32_MIN;
  int32_t dcMin = INT32_MAX;
  int32_t dcMax = INT32_MIN;
  float normMax = 0.0f;

  const unsigned long start = millis();
  while (millis() - start < windowMs)
  {
    const int32_t filtered = *gDeps.filteredMic;
    const int32_t dc = *gDeps.micDc;
    const float norm = *gDeps.normalizedMic;

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

    updateMalletsTick();
  }

  const int32_t filteredP2P = (filteredMax > filteredMin) ? (filteredMax - filteredMin) : 0;

  gDeps.log->print("micprobe ms=");
  gDeps.log->print(windowMs);
  gDeps.log->print(" ch=");
  gDeps.log->print((*gDeps.micChannelSelect == gDeps.micChannelRightValue) ? "right" : "left");
  gDeps.log->print(" leftPeak=");
  gDeps.log->print(*gDeps.micLeftPeak);
  gDeps.log->print(" rightPeak=");
  gDeps.log->print(*gDeps.micRightPeak);
  gDeps.log->print(" filtMin=");
  gDeps.log->print(filteredMin);
  gDeps.log->print(" filtMax=");
  gDeps.log->print(filteredMax);
  gDeps.log->print(" filtP2P=");
  gDeps.log->print(filteredP2P);
  gDeps.log->print(" dcMin=");
  gDeps.log->print(dcMin);
  gDeps.log->print(" dcMax=");
  gDeps.log->print(dcMax);
  gDeps.log->print(" normMax=");
  gDeps.log->println(normMax, 3);
}

void setMicChannelFromCommand(const String &cmd)
{
  if (!dependenciesReady())
  {
    return;
  }

  int channel = -1;
  if (cmd == "micch left" || cmd == "micch l" || cmd == "micch 0")
  {
    channel = gDeps.micChannelLeftValue;
  }
  else if (cmd == "micch right" || cmd == "micch r" || cmd == "micch 1")
  {
    channel = gDeps.micChannelRightValue;
  }

  if (channel < 0)
  {
    gDeps.log->println("micch usage: micch left|right (or 0|1)");
    return;
  }

  *gDeps.micChannelSelect = channel;
  gDeps.log->print("Mic channel set to ");
  gDeps.log->println((*gDeps.micChannelSelect == gDeps.micChannelRightValue) ? "right" : "left");
}

void setMicShiftFromCommand(const String &cmd)
{
  if (!dependenciesReady())
  {
    return;
  }

  const int separator = cmd.indexOf(' ');
  if (separator < 0 || separator + 1 >= static_cast<int>(cmd.length()))
  {
    gDeps.log->print("micshift usage: micshift <0..");
    gDeps.log->print(gDeps.micGainShiftMaxBits);
    gDeps.log->println(">");
    return;
  }

  int shift = cmd.substring(separator + 1).toInt();
  if (shift < 0)
  {
    shift = 0;
  }
  if (shift > gDeps.micGainShiftMaxBits)
  {
    shift = gDeps.micGainShiftMaxBits;
  }

  *gDeps.micGainShiftBits = shift;
  *gDeps.micLeftPeak = 0;
  *gDeps.micRightPeak = 0;
  *gDeps.micDc = 0;
  gDeps.log->print("Mic gain shift set to ");
  gDeps.log->print(*gDeps.micGainShiftBits);
  gDeps.log->print(" (24-bit decode shift fixed at ");
  gDeps.log->print(gDeps.micSlotTo24BitShiftBits);
  gDeps.log->println(")");
}

void setMicModeFromCommand(const String &cmd)
{
  if (!dependenciesReady())
  {
    return;
  }

  int mode = -1;
  if (cmd == "micmode auto" || cmd == "micmode a" || cmd == "micmode 0")
  {
    mode = gDeps.micSampleModeAutoValue;
  }
  else if (cmd == "micmode stereo" || cmd == "micmode s" || cmd == "micmode 1")
  {
    mode = gDeps.micSampleModeStereoValue;
  }
  else if (cmd == "micmode mono" || cmd == "micmode m" || cmd == "micmode 2")
  {
    mode = gDeps.micSampleModeMonoValue;
  }
  else
  {
    gDeps.log->println("micmode usage: micmode auto|stereo|mono (or 0|1|2)");
    return;
  }

  *gDeps.micSampleMode = mode;
  gDeps.log->print("Mic sample mode set to ");
  gDeps.log->println(micSampleModeToString(*gDeps.micSampleMode));
}

} // namespace AudioDiagnostics
