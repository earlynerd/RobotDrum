#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <Arduino.h>

namespace SerialCommands
{

struct Handlers
{
  void (*playCalibrationArpeggio)();
  void (*runFullCalibration)();
  void (*runFullCalibrationForMallet)(size_t index);
  void (*runVelocityCalibrationOnly)();
  void (*printMalletStatus)();
  void (*printAudioStatus)();
  void (*printMicDiagnostics)();
  void (*runMicProbe)(const String &cmd);
  void (*setMicChannelFromCommand)(const String &cmd);
  void (*setMicShiftFromCommand)(const String &cmd);
  void (*setMicModeFromCommand)(const String &cmd);
  void (*runFftTestForMallet)(int index);
  void (*setMalletMidiFromCommand)(const String &cmd);
  void (*setMalletStrikeFromCommand)(const String &cmd);
  void (*setMalletReboundFromCommand)(const String &cmd);
  void (*printLoopStats)();
  void (*runReboundCalibration)();
  void (*runReboundCalibrationForMallet)(size_t index);
  void (*toggleVolumeBalance)();
};

void dispatch(const String &input, const Handlers &handlers, size_t malletCount, Stream &out);

} // namespace SerialCommands

#endif
