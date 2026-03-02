#ifndef __MALLET_H
#define __MALLET_H

#include <Arduino.h>

#define StrikeQueueDepth    10

class Mallet{
  public:
  struct StrikeProfile {
    int strikePower;
    int strikeDuration;
    int coastPower;
    int coastDuration;
    int reboundPower;
    int reboundDuration;
  };

  Mallet(int pin, int MIDIpitch, int malletChannel, int strikePWM, int strikeTime, int coastPWM, int coastTime, int reboundPWM, int reboundTime);
  int pin;
  int channel;
  int MIDInote;
  int strikePower;
  int strikeDuration;
  int coastDuration;
  int coastPower;
  int reboundDuration;
  int reboundPower;

  void updateMallet();
  void triggerMallet();
  void delayedTrigger(unsigned long delayTime);
  void delayedTrigger(unsigned long delayTime, const StrikeProfile &profile);
  bool isIdle();
  void setDelay(unsigned long lagTime);
  void setRetriggerGap(unsigned long gapMs);
  void abortAndClearQueue();
  int getMidiPitch();
  void setMidiPitch(int pitch);
  unsigned long getDelay();
  unsigned long getRetriggerGap();
  
  enum State {
  IDLESTATE=0,
  TRIGGER,
  STRIKE,
  COAST,
  REBOUND
};

  
  private:
  unsigned long lag;                              //the measured lag between the stick trigger and the stick contact with drum. calibrated by a routine in loop()
  unsigned long delayedStrikeTriggerTimes[StrikeQueueDepth] = {};    //the calculated millis() value when the queued strike should occur
  StrikeProfile delayedStrikeProfiles[StrikeQueueDepth] = {};
  bool delayedStrikeHasProfile[StrikeQueueDepth] = {};
  uint16_t queuedStrikeCount = 0;                //keep track of the number of delayed strikes queued
  void handleQueuedStrikes();
  unsigned long minRetriggerGapMs = 0;
  unsigned long lastCycleEndMillis = 0;

  State malletState = IDLESTATE;

  unsigned long triggerTime;
  int malletPin;
};
#endif
