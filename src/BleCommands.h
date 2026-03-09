#ifndef BLE_COMMANDS_H
#define BLE_COMMANDS_H

#include <BLEServer.h>
#include "SerialCommands.h"

namespace BleCommands
{

struct Config
{
  BLEServer *pServer;
  SerialCommands::Handlers handlers;
  size_t malletCount;
  Stream **commandOut;  // pointer to the active output stream pointer
};

void initialize(const Config &config);
void poll();
void setConnected(bool connected);

} // namespace BleCommands

#endif
