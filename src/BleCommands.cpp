#include "BleCommands.h"

#include <BLEDevice.h>
#include <BLE2902.h>

namespace
{

// Custom service UUID for the command channel (alongside the MIDI service).
constexpr const char *kCmdServiceUuid = "e3a10001-ede8-4b33-a751-6ce34ec4c700";
constexpr const char *kCmdCharUuid = "e3a10002-ede8-4b33-a751-6ce34ec4c700";

constexpr size_t kCmdQueueSize = 128;
constexpr size_t kResponseBufSize = 2048;
constexpr size_t kNotifyChunkSize = 500;

// ── Command queue (written by BLE callback, read by loop) ───────

char gCmdQueue[kCmdQueueSize];
volatile size_t gCmdLen = 0;
portMUX_TYPE gCmdMux = portMUX_INITIALIZER_UNLOCKED;

// ── Stored config ───────────────────────────────────────────────

BleCommands::Config gConfig = {};
BLECharacteristic *gCmdChar = nullptr;
bool gBleClientConnected = false;

// ── BleStream: captures print() output into a buffer ────────────

class BleStream : public Stream
{
public:
  void reset() { pos = 0; }

  size_t write(uint8_t c) override
  {
    if (pos < kResponseBufSize)
    {
      buf[pos++] = c;
    }
    return 1;
  }

  size_t write(const uint8_t *data, size_t len) override
  {
    for (size_t i = 0; i < len; ++i)
    {
      write(data[i]);
    }
    return len;
  }

  // Send buffered content as BLE notifications.
  void sendResponse()
  {
    if (pos == 0 || gCmdChar == nullptr || !gBleClientConnected)
    {
      return;
    }

    size_t sent = 0;
    while (sent < pos)
    {
      const size_t chunk = ((pos - sent) < kNotifyChunkSize) ? (pos - sent) : kNotifyChunkSize;
      gCmdChar->setValue(buf + sent, chunk);
      gCmdChar->notify();
      sent += chunk;
      if (sent < pos)
      {
        vTaskDelay(pdMS_TO_TICKS(20));
      }
    }
  }

  // Stream interface stubs (not used for input).
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }

private:
  uint8_t buf[kResponseBufSize];
  size_t pos = 0;
};

BleStream gBleStream;

// ── BLE callbacks ───────────────────────────────────────────────

class CmdCharCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic) override
  {
    std::string val = pCharacteristic->getValue();
    if (val.empty())
    {
      return;
    }

    portENTER_CRITICAL(&gCmdMux);
    if (gCmdLen == 0 && val.size() < kCmdQueueSize)
    {
      memcpy(gCmdQueue, val.c_str(), val.size());
      gCmdLen = val.size();
    }
    portEXIT_CRITICAL(&gCmdMux);
  }
};

// Connection state is tracked via setConnected() called from the main
// module's BLE MIDI connect/disconnect callbacks, since the ESP32 BLE
// API only supports one BLEServerCallbacks per server.

} // namespace

namespace BleCommands
{

void initialize(const Config &config)
{
  gConfig = config;

  BLEServer *pServer = config.pServer;
  if (pServer == nullptr)
  {
    return;
  }

  gBleClientConnected = false;

  // The caller must ensure advertising has NOT been started yet.
  // All BLE services must be registered before advertising begins
  // on ESP32 Bluedroid, or the GATT table will be corrupted.
  BLEService *pService = pServer->createService(BLEUUID(kCmdServiceUuid));
  gCmdChar = pService->createCharacteristic(
    BLEUUID(kCmdCharUuid),
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  gCmdChar->addDescriptor(new BLE2902());
  gCmdChar->setCallbacks(new CmdCharCallback());
  pService->start();
}

void poll()
{
  portENTER_CRITICAL(&gCmdMux);
  const size_t len = gCmdLen;
  char cmd[kCmdQueueSize];
  if (len > 0)
  {
    memcpy(cmd, gCmdQueue, len);
    gCmdLen = 0;
  }
  portEXIT_CRITICAL(&gCmdMux);

  if (len == 0)
  {
    return;
  }

  const String command(cmd, len);
  gBleStream.reset();

  // Redirect handler output to the BLE stream for this command.
  Stream *prevOut = (gConfig.commandOut != nullptr) ? *gConfig.commandOut : nullptr;
  if (gConfig.commandOut != nullptr)
  {
    *gConfig.commandOut = &gBleStream;
  }

  SerialCommands::dispatch(command, gConfig.handlers, gConfig.malletCount, gBleStream);
  gBleStream.sendResponse();

  if (gConfig.commandOut != nullptr)
  {
    *gConfig.commandOut = prevOut;
  }
}

void setConnected(bool connected)
{
  gBleClientConnected = connected;
}

} // namespace BleCommands
