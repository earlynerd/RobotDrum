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
int bufferLength = 256;
int sampleRate = 8000;

const int fftLength = 4096;
float fft_input[fftLength];
float fft_output[fftLength];
float fftFrequency = 0;
float fftMag = 0;

int midiNote[] = {8, 9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 19, 21, 22, 23, 24, 26, 28, 29, 31, 33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902, 8372, 8870, 9397, 9956, 10548, 11175, 11840, 12544};

bool doFFT = false;
bool newFFTData = false;

i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = bufferLength,
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
Mallet note_1 (malletPins[0], 0x34, 0, 900, 110, 0, 50, 512, 50); //162Hz midi 52
Mallet note_2 (malletPins[1], 0x37, 1, 900, 110, 0, 50, 512, 50); //192Hz midi 55
Mallet note_3 (malletPins[2], 0x39, 2, 900, 110, 0, 50, 512, 50); //220Hz midi 57
Mallet note_4 (malletPins[3], 0x3C, 3, 900, 110, 0, 50, 512, 50); //257Hz midi 60
Mallet note_5 (malletPins[4], 0x3E, 4, 900, 110, 0, 50, 512, 50); //288Hz midi 62

Mallet note_6 (malletPins[5], 0x40, 5, 900, 110, 0, 50, 512, 50); //324Hz midi 64
Mallet note_7 (malletPins[6], 0x41, 6, 900, 110, 0, 50, 512, 50); //343Hz midi 65
Mallet note_8 (malletPins[7], 0x43, 7, 900, 110, 0, 50, 512, 50); //385Hz midi 67
Mallet note_9 (malletPins[8], 0x45, 8, 900, 110, 0, 50, 512, 50); //432Hz midi 69
Mallet note_10(malletPins[9], 0x48, 9, 900, 110, 0, 50, 512, 50); //513Hz midi 72
Mallet *notes[] = {&note_1, &note_2, &note_3, &note_4, &note_5, &note_6, &note_7, &note_8, &note_9, &note_10};

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
void updateMalletParameters(Mallet *mallet, unsigned long strikeDelay, int pitch);
unsigned long recallStoredCalibration(Mallet *mallet, int index);
unsigned long updateStoredCalibration(Mallet *mallet, int index);
void calculateFFT(int32_t *buf, int32_t bufsize);
int identifyNote();
void delayForRingdown(float percent);
void sortMalletsByPitch();

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  EEPROM.begin((size_t)8 * (numMallets + 1));
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
  BLEMidiServer.setOnConnectCallback([]()
                                     { Serial.println("Connected!"); });
  BLEMidiServer.setOnDisconnectCallback([]() { // To show how to make a callback with a lambda function
    Serial.println("Disconnected");
  });
  BLEMidiServer.setNoteOnCallback(handleNoteOn);
  BLEMidiServer.setNoteOffCallback(handleNoteOff);

  //init microphone and create task
  i2sSampler = new I2SMEMSSampler(i2sPins, false);
  TaskHandle_t i2sMemsWriterTaskHandle;
  xTaskCreatePinnedToCore(i2sMemsWriterTask, "I2S Writer Task", 8192, i2sSampler, 1, &i2sMemsWriterTaskHandle, 1);
  //i2sSampler->start(I2S_NUM_1, i2s_config, (int32_t)blockSize, i2sMemsWriterTaskHandle);
  i2sSampler->start(I2S_NUM_1, i2s_config, bufferLength * 4, i2sMemsWriterTaskHandle);
  digitalWrite(13, LOW);
  delay(4000);
  calibrateMallets();
  /*
  Serial.println("Recalling stored mallet calibration data");
  unsigned long writeCount;
  for (int i = 0; i < numMallets; i++)
  {
    writeCount = recallStoredCalibration(notes[i], i);
  }
  Serial.print("EEPROM Writes: ");
  Serial.println(writeCount);
  sortMalletsByPitch();
  */
}

void loop()
{
  malletUpdate();
  //identifyNote();
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
  if (timestamp < lastTimestamp)
  { //timestamp rollover has occurred
    lastRollovermillis = millis();
    lastRolloverTimestamp = timestamp; //first one after an apparent timestamp rollover
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
      unsigned long compensation = (thisNotemillis - notes[i]->getDelay()) - millis();
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
      calculateFFT(buf, bufsize);
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
  //sort notes[] according to ascending pitch, as detected
  sortMalletsByPitch();
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
  malletDelay(800);        
  delayForRingdown(0.20);
  Serial.print("Max Microphone: ");
  Serial.println(maxMicrophone);
  malletDelay(400);

  mallet->strikeDuration = 500;                       //second strike to get initial idea of strikedelay
  unsigned long strikeDelay = testMalletLag(mallet);
  delayForRingdown(0.25);
  malletDelay(800);
  Serial.print("Strike Delay measured: ");
  Serial.println(strikeDelay);
  updateMalletParameters(mallet, strikeDelay, -1);
  delayForRingdown(0.35);
  Serial.print("attempt 1 Strike Duration: ");
  Serial.println(mallet->strikeDuration);
  malletDelay(500);

  Serial.println("Verifying parameters, and detecting pitch");      //third strike to dial in delay and detect note pitch
  strikeDelay = testMalletLag(mallet);
  Serial.print("Final Strike Delay: ");
  Serial.println(strikeDelay);
  int thisNote = identifyNote();
  float thisFreq = fftFrequency;
  float thisMag = fftMag;
  Serial.print("Midi Pitch Detected: ");
  Serial.print(thisNote);
  Serial.print(", Frequency: ");
  Serial.print(thisFreq);
  Serial.print(", Magnitude: ");
  Serial.println(thisMag);
  updateMalletParameters(mallet, strikeDelay, thisNote);
  delayForRingdown(0.20);
  malletDelay(1000);
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
  delayForRingdown(0.20);
  unsigned long startTime = micros();
  mallet->triggerMallet(); //strike and measure
  while ((normalizedMicrophone < 0.60) && (micros() - startTime < 4000000))
  {                 //run loop until sound of mallet striking drum is heard. or 4 secoonds regardless
    malletUpdate(); // maxMicrophone and normalizedMicrophone should automatically update.
  }
  unsigned long strikeTime = micros();
  unsigned long strikeDelay = (unsigned long)(((float)strikeTime - (float)startTime) / 1000.0); //milliseconds
  
  return strikeDelay;
}

void delayForRingdown(float percent)
{
  unsigned long entryTime = millis();
  const unsigned long timeout = 5000;
  while ((normalizedMicrophone > percent) && (millis() - entryTime < timeout))
  {
    malletUpdate(); //delay until rung down
  } 
}

void updateMalletParameters(Mallet *mallet, unsigned long strikeDelay, int pitch)
{
  mallet->strikeDuration = (int)((float)strikeDelay * 1.6);
  mallet->coastDuration = (int)((float)strikeDelay * 0.1);
  mallet->reboundPower = 100;
  mallet->reboundDuration = (int)((float)strikeDelay * 1.5);
  if(pitch > 0)
  {
    mallet->setMidiPitch(pitch);
  }
}

unsigned long updateStoredCalibration(Mallet *mallet, int index)
{
  unsigned long writeCount = EEPROM.readULong(0);
  writeCount++;
  EEPROM.writeULong(0, writeCount);
  Serial.print("Writing Mallet ");
  Serial.print(index);
  Serial.print(" Strike Delay: ");
  Serial.print(mallet->getDelay());
  Serial.println(" ms");
  EEPROM.writeULong((index + 1) * 2 * sizeof(unsigned long), mallet->getDelay());
  EEPROM.writeLong((index + 1) * 2 * sizeof(unsigned long) + sizeof(int32_t), (int32_t)mallet->getMidiPitch());
  if (EEPROM.commit())
  {
    Serial.println("EEPROM Write Success\n");
    return writeCount;
  }
  else
    return 0;
}

unsigned long recallStoredCalibration(Mallet *mallet, int index)
{
  unsigned long writeCount = EEPROM.readULong(0);
  if (writeCount > 0)
  {
    unsigned long temp = EEPROM.readULong((index + 1) * 2 * sizeof(unsigned long));
    int pitch = EEPROM.readLong((index + 1) * 2 * sizeof(unsigned long) + sizeof(int32_t));
    mallet->setDelay(temp);
    updateMalletParameters(mallet, temp, pitch);
    Serial.print("Mallet ");
    Serial.print(index);
    Serial.print(" Strike Delay: ");
    Serial.print(temp);
    Serial.print("ms, Pitch: ");
    Serial.println(pitch);
  }
  else
  {
    Serial.println("No calibration data found");
  }
  return writeCount;
}

void calculateFFT(int32_t *buf, int32_t bufsize)
{
  static int sampleCounter = 0;
  int32_t index = 0;
  while (index < bufsize)
  {
    fft_input[sampleCounter] = (float)buf[index];
    index++;
    sampleCounter++;
    if (sampleCounter >= fftLength)
    {
      float totalTime = (float)fftLength / (float)sampleRate;
      float max_magnitude = 0;
      float fundamental_freq = 0;

      fft_config_t *real_fft_plan = fft_init(fftLength, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

      //long int t1 = micros();
      fft_execute(real_fft_plan);
      for (int k = 1; k < real_fft_plan->size / 2; k++)
      {
        /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
        float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2)) / 1;
        float freq = k * 1.0 / totalTime;
        //    sprintf(print_buf,"%f Hz : %f", freq, mag);
        //    Serial.println(print_buf);
        if (mag > max_magnitude)
        {
          max_magnitude = mag;
          fundamental_freq = freq;
        }
      }
      //long int t2 = micros();

      //Serial.println();
      /*Multiply the magnitude of the DC component with (1/FFT_N) to obtain the DC component*/
      //sprintf(print_buf,"DC component : %f g\n", (real_fft_plan->output[0])/10000/bufferLength);  // DC is at [0]
      //Serial.println(print_buf);

      /*Multiply the magnitude at all other frequencies with (2/FFT_N) to obtain the amplitude at that frequency*/
      //sprintf(print_buf,"Fundamental Freq : %f Hz\t Mag: %f g\n", fundamental_freq, (max_magnitude/10000)*2/bufferLength);
      //Serial.println(print_buf);
      float magn = (max_magnitude / 10000) * 2 / fftLength;
      fftFrequency = fundamental_freq;
      fftMag = magn;
      newFFTData = true;
      //if (magn > 1000)
      //{
      //Serial.print("Fundamental:");
      //Serial.print(fundamental_freq);
      //Serial.print(", ");
      //Serial.print("Magnitude:");
      //Serial.print(magn);
      //Serial.println();
      //}

      //Serial.print("Time taken: ");Serial.print((t2-t1)*1.0/1000);Serial.println(" milliseconds!");

      fft_destroy(real_fft_plan);
      sampleCounter = 0;
    }
  }
}

int identifyNote()
{
  doFFT = true;
  newFFTData = false;
  static int lastNote = -1;
  int note = -1;
  while(!newFFTData) malletUpdate();   //wait for result to come in
  doFFT = false;
  newFFTData = false;
  //if (fftMag > 1000)
  //{
    int index = 1;
    int lengthMidiArray = sizeof(midiNote) / sizeof(midiNote[0]);
    if (fftFrequency < midiNote[0])
    {
      note = 0;
    }
    else if (fftFrequency > midiNote[lengthMidiArray - 1])
    {
      note = lengthMidiArray - 1;
    }
    else
    {
      while (index < (lengthMidiArray ) && (note == -1))
      {
        if (fftFrequency < (float)midiNote[index])
        {
          if (abs(fftFrequency - (float)midiNote[index]) < abs(fftFrequency - (float)midiNote[index - 1]))
          {
            note = index;
          }
          else
          {
            note = index - 1;
          }
        }
        index++;
      }
    }
    if(note != lastNote)
    {
      Serial.print("Note:");
      Serial.println(note);
      lastNote = note;
    }
    doFFT = false;
    return note;
  //}
  doFFT = false;
  return note;
}

void sortMalletsByPitch()
{
  bool done = false;
  while(!done)
  {
    //loop over iteration and swap out-of-order pairs
    for(int k = 0; k < numMallets-1; k++)
    {
      if(notes[k]->getMidiPitch() > notes[k+1]->getMidiPitch()){
        Mallet* temp = notes[k];
        notes[k] = notes[k+1];
        notes[k+1] = temp;
      }
    }
    //check if in ascending order
    bool sorted = true;
    for(int i = 0; i < numMallets-1; i++){
      if(notes[i]->getMidiPitch() > notes[i+1]->getMidiPitch()) sorted = false;
    }
    if(sorted) done = true; 
  }
}