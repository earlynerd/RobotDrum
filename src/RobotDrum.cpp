#include <WiFi.h>
#include "driver/i2s.h"
#include <Arduino.h>
#include "I2SMEMSSampler.h"
#include "mallet.h"
//#include "MIDI.h"
#include <BLEMidi.h>
#include <EEPROM.h>
#include <fft.h>

I2SSampler *i2sSampler = NULL;

i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 48000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 200,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

i2s_pin_config_t i2sPins = {
    .bck_io_num = 5,                   // BCK  (SCK)    - bit clock or serial clock
    .ws_io_num = 9,                    // LRCK (WS, FS) - left-right clock or word select or frame sync
    .data_out_num = I2S_PIN_NO_CHANGE, // DATA output   - not used
    .data_in_num = 10                  // DIN  (SD)     - serial data in  (SDATA, SDIN, SDOUT, DACDAT, ADCDAT)
};

int32_t filteredMicrophone = 0;
int32_t maxMicrophone = 0, minMicrophone = 80000000;
float normalizedMicrophone = 0;

/*
//MIDI_CREATE_DEFAULT_INSTANCE();
struct MySettings : public midi::DefaultSettings
{
  static const long BaudRate = 115200;
};


MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MySettings);
*/

const unsigned long midiLeadTime = 1000; //midi commands delayed by this time in order to add compensation for strike times
//  1 2 3  4  5  6  7  8  9  10   placement
//  2 8 1  9  3  7  5  10 4  6    relative pitch
//  4 2 15 13 12 14 27 33 32 26   pin number
int malletPins[] = {15, 4, 12, 32, 27, 26, 25, 2, 13, 33}; //ascending note order
//int malletPins[] = {15, 4, 12, 35, 36, 26, 14, 2, 15, 34}; //ascending note order

//longest tongue toward the leftmost mallet (number 3)

//shortest tongue toward the rightmost mallet (number 8)
const int numMallets = 10;
//pin, channel, strikePower, strikeDuration, coastPower, coastDuration, reboundPower, reboundDuration
Mallet note_E3(malletPins[0], 0x34, 0, 900, 110, 0, 50, 512, 50); //162Hz
Mallet note_G3(malletPins[1], 0x37, 1, 900, 110, 0, 50, 512, 50); //192Hz
Mallet note_A3(malletPins[2], 0x39, 2, 900, 110, 0, 50, 512, 50); //220Hz
Mallet note_C4(malletPins[3], 0x3C, 3, 900, 110, 0, 50, 512, 50); //257Hz
Mallet note_D4(malletPins[4], 0x3E, 4, 900, 110, 0, 50, 512, 50); //288Hz

Mallet note_E4(malletPins[5], 0x40, 5, 900, 110, 0, 50, 512, 50); //324Hz
Mallet note_F4(malletPins[6], 0x41, 6, 900, 110, 0, 50, 512, 50); //343Hz
Mallet note_G4(malletPins[7], 0x43, 7, 900, 110, 0, 50, 512, 50); //385Hz
Mallet note_A4(malletPins[8], 0x45, 8, 900, 110, 0, 50, 512, 50); //432Hz
Mallet note_C5(malletPins[9], 0x48, 9, 900, 110, 0, 50, 512, 50); //513Hz
Mallet *notes[] = {&note_E3, &note_G3, &note_A3, &note_C4, &note_D4, &note_E4, &note_F4, &note_G4, &note_A4, &note_C5};

//void handleNoteOff(byte channel, byte pitch, byte velocity);
void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp);
//void handleNoteOn(byte channel, byte pitch, byte velocity);
void handleNoteOn(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp);
void malletUpdate();
void i2sMemsWriterTask(void *param);
unsigned long testStrike(Mallet *mallet);
void calibrateMallets();
void malletDelay(uint32_t milliseconds);
unsigned long testMalletLag(Mallet *mallet);
int32_t emaFilter(int32_t in, int32_t average, float alpha);
void updateMalletParameters(Mallet *mallet, unsigned long strikeDelay);
unsigned long recallStoredCalibration(Mallet* mallet, int index);
unsigned long updateStoredCalibration(Mallet* mallet, int index);


void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  EEPROM.begin((size_t)4 * (numMallets + 1));
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  // Connect the handleNoteOn function to the library,
  // so it is called upon reception of a NoteOn.
  //MIDI.setHandleNoteOn(handleNoteOn); // Put only the name of the function

  // Do the same for NoteOffs
  //MIDI.setHandleNoteOff(handleNoteOff);

  // Initiate MIDI communications, listen to all channels
  //MIDI.begin(MIDI_CHANNEL_OMNI);

  BLEMidiServer.begin("MusicRobot");
  BLEMidiServer.setOnConnectCallback([]() {
    Serial.println("Connected!");
  });
  BLEMidiServer.setOnDisconnectCallback([]() { // To show how to make a callback with a lambda function
    Serial.println("Disconnected");
  });
  BLEMidiServer.setNoteOnCallback(handleNoteOn);
  BLEMidiServer.setNoteOffCallback(handleNoteOff);

  //init microphone and create task
  i2sSampler = new I2SMEMSSampler(i2sPins, false);
  TaskHandle_t i2sMemsWriterTaskHandle;
  xTaskCreatePinnedToCore(i2sMemsWriterTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsWriterTaskHandle, 1);
  //i2sSampler->start(I2S_NUM_1, i2s_config, (int32_t)blockSize, i2sMemsWriterTaskHandle);
  i2sSampler->start(I2S_NUM_1, i2s_config, 200, i2sMemsWriterTaskHandle);
  digitalWrite(13, LOW);
  delay(4000);
  //calibrateMallets();
  Serial.println("Recalling stored mallet calibration data");
  unsigned long writeCount;
  for(int i = 0; i < numMallets; i++){
    writeCount = recallStoredCalibration(notes[i], i);
  }
  Serial.print("EEPROM Writes: ");
  Serial.println(writeCount);
  
}

void loop()
{
  malletUpdate();
  //MIDI.read();
}

void malletUpdate()
{
  for (int i = 0; i < numMallets; i++)
  {
    notes[i]->updateMallet();
  }
}

//void handleNoteOn(byte channel, byte pitch, byte velocity)
void handleNoteOn(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp)
{
  static unsigned long lastRollovermillis = 0, lastRolloverTimestamp = 0;    
  static unsigned long lastTimestamp = 9000;
  if(timestamp < lastTimestamp){      //timestamp rollover has occurred
    lastRollovermillis = millis();
    lastRolloverTimestamp = timestamp;    //first one after an apparent timestamp rollover
  }
  unsigned long realtimeSinceLastRolloverDetected = millis() - lastRollovermillis;
  unsigned long lastDetectedIntervalStartmillis = millis() - realtimeSinceLastRolloverDetected - lastRolloverTimestamp;
  unsigned long rolloversSinceThen = (millis() - lastDetectedIntervalStartmillis) / 8192;
  unsigned long lastIntervalStartMillis = lastDetectedIntervalStartmillis + (rolloversSinceThen * 8192); 
  unsigned long thisNotemillis = lastIntervalStartMillis + timestamp;
  thisNotemillis += 1000;
  for (int i = 0; i < 10; i++)
  {
    if (notes[i]->MIDInote == note)
    {
      Serial.print("note triggered: ");
      Serial.print(note);
      Serial.print(", timestamped: ");
      Serial.print(timestamp);
      Serial.print(", milliseconds: ");
      Serial.println(millis());
      //notes[i]->triggerMallet();
      //unsigned long compensation = midiLeadTime - notes[i]->getDelay();
      unsigned long compensation = (thisNotemillis - notes[i]->getDelay()) -  millis();
      notes[i]->delayedTrigger(compensation);
    }
  }
  lastTimestamp = timestamp;
}

//void handleNoteOff(byte channel, byte pitch, byte velocity)
void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp)
{
  // Do something when the note is released.
  // Note that NoteOn messages with 0 velocity are interpreted as NoteOffs.
}

void i2sMemsWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(300);
  while (true)
  {
    // wait for some samples to save
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      int32_t *buf = sampler->getCapturedAudioBuffer();
      int32_t bufsize = sampler->getBufferSizeInBytes() / 4;
      for (int32_t i = 0; i < bufsize; i++)
      {
        uint32_t sample = abs(buf[i]);
        filteredMicrophone = emaFilter(sample, filteredMicrophone, 0.01);
        //Serial.println(filteredMicrophone);
        if (filteredMicrophone > maxMicrophone)
          maxMicrophone = filteredMicrophone;
        if (filteredMicrophone < minMicrophone)
          minMicrophone = filteredMicrophone;
        normalizedMicrophone = ((float)filteredMicrophone - (float)minMicrophone) / ((float)maxMicrophone - (float)minMicrophone);
        if (normalizedMicrophone > 0.5)
          digitalWrite(13, HIGH);
        else
          digitalWrite(13, LOW);
      }
    }
  }
}

int32_t emaFilter(int32_t in, int32_t average, float alpha)
{
  int64_t temp;
  uint16_t integerAlpha = (uint16_t)(alpha * 65536);
  temp = ((int64_t)in * integerAlpha) + ((int64_t)average * (65536 - integerAlpha));
  return (int32_t)((temp + 32768) / 65536);
}

void calibrateMallets()
{ //todo: add save to eeprom
  //for each mallet:
  //do a test strike. use peak value of filtered mic to normalize subsequent readings.
  //do a calibration strike. record time delay. calculate timing for input shaping
  //do a test strike using new parameters.
  //clear normalization and move to next mallet.
  for (int i = 0; i < numMallets; i++)
  {
    Serial.print("Calibrating Mallet ");
    Serial.println(i + 1);
    unsigned long malletLag = testStrike(notes[i]);
    notes[i]->setDelay(malletLag);
    updateStoredCalibration(notes[i], i);
    malletDelay(500);
  }
  
  //do a scale up and down to demonstrate
  for (int i = 0; i < numMallets; i++)
  {
    notes[i]->triggerMallet();
    malletDelay(200);
  }
  malletDelay(400);
  for (int i = numMallets - 1; i >= 0; i--)
  {
    notes[i]->triggerMallet();
    malletDelay(200);
  }
  malletDelay(3000);
}

unsigned long testStrike(Mallet *mallet)
{
  mallet->strikePower = 1023;
  mallet->strikeDuration = 400;

  mallet->coastPower = 0;
  mallet->coastDuration = 1;

  mallet->reboundPower = 0;
  mallet->reboundDuration = 1;

  maxMicrophone = 0;
  Serial.println("Striking to normalize mic feedback");
  mallet->triggerMallet(); //first strike to normalize mic readings
  malletDelay(800);        //should now be normalized to detect timing for real.
  while (normalizedMicrophone > 0.20)
    malletUpdate(); //delay for ring down

  Serial.print("Max Microphone: ");
  Serial.println(maxMicrophone);

  malletDelay(400);

  mallet->strikeDuration = 500;
  unsigned long strikeDelay = testMalletLag(mallet);

  Serial.print("Strike Delay measured: ");
  Serial.println(strikeDelay);

  updateMalletParameters(mallet, strikeDelay);

  while (normalizedMicrophone > 0.35)
    malletUpdate(); //delay for ring down

  Serial.print("attempt 1 Strike Duration: ");
  Serial.println(mallet->strikeDuration);

  malletDelay(500);

  Serial.println("Verifying parameters");
  strikeDelay = testMalletLag(mallet);

  updateMalletParameters(mallet, strikeDelay);

  Serial.print("Final Strike Delay: ");
  Serial.println(strikeDelay);

  malletDelay(800);
  return strikeDelay;
}

void malletDelay(uint32_t milliseconds)
{
  unsigned long startTime = millis();
  while (millis() - startTime < milliseconds)
  {
    malletUpdate();
  }
}

unsigned long testMalletLag(Mallet *mallet)
{
  malletDelay(500);
  while (normalizedMicrophone > 0.20)
    malletUpdate(); //delay for ring down
  unsigned long startTime = micros();
  mallet->triggerMallet(); //strike and measure
  while ((normalizedMicrophone < 0.60) && (micros() - startTime < 4000000))
  {                 //run loop until sound of mallet striking drum is heard. or 4 secoonds regardless
    malletUpdate(); // maxMicrophone and normalizedMicrophone should automatically update.
  }
  unsigned long strikeTime = micros();
  unsigned long strikeDelay = (unsigned long)(((float)strikeTime - (float)startTime) / 1000.0); //milliseconds
  while (normalizedMicrophone > 0.25)
    malletUpdate(); //delay until rung down
  malletDelay(800);
  return strikeDelay;
}

void updateMalletParameters(Mallet *mallet, unsigned long strikeDelay)
{
  mallet->strikeDuration = (int)((float)strikeDelay * 1.6);
  mallet->coastDuration = (int)((float)strikeDelay * 0.1);
  mallet->reboundPower = 100;
  mallet->reboundDuration = (int)((float)strikeDelay * 1.5);
}

unsigned long updateStoredCalibration(Mallet* mallet, int index)
{
  unsigned long writeCount = EEPROM.readULong(0);
  writeCount++;
  EEPROM.writeULong(0, writeCount);
  Serial.print("Writing Mallet ");
  Serial.print(index);
  Serial.print(" Strike Delay: ");
  Serial.print( mallet->getDelay());
  Serial.println(" ms");
  EEPROM.writeULong((index+1) * sizeof(unsigned long), mallet->getDelay());
  if (EEPROM.commit()){
    Serial.println("EEPROM Write Success\n");
    return writeCount;
  }
  else
    return 0;
}

unsigned long recallStoredCalibration(Mallet* mallet, int index)
{
  unsigned long writeCount = EEPROM.readULong(0);
  if (writeCount > 0)
  {
    unsigned long temp = EEPROM.readULong((index+1) * sizeof(unsigned long));
    mallet->setDelay(temp);
    updateMalletParameters(mallet, temp);
    Serial.print("Mallet ");
    Serial.print(index);
    Serial.print(" Strike Delay: ");
    Serial.print(temp);
    Serial.println(" ms");
  }
  else{
    Serial.println("No calibration data found");
  }
  return writeCount;
}