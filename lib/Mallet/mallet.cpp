#include "Mallet.h"

Mallet::Mallet(int pin, int MIDIpitch, int malletChannel, int strikePWM, int strikeTime, int coastPWM, int coastTime, int reboundPWM, int reboundTime)
{
  strikePower = strikePWM;
  strikeDuration = strikeTime;
  coastPower = coastPWM;
  coastDuration = coastTime;
  reboundPower = reboundPWM;
  reboundDuration = reboundTime;
  malletPin = pin;
  MIDInote = MIDIpitch;
  channel = malletChannel;
  ledcAttachPin(malletPin, channel);
  ledcSetup(channel, 18000, 10);
}

void Mallet::updateMallet()
{
  handleQueuedStrikes();

  switch (malletState)
  {
  case IDLESTATE:
    ledcWrite(channel, 0);
    break;

  case TRIGGER:
    triggerTime = millis();
    malletState = STRIKE;
    break;

  case STRIKE:
    ledcWrite(channel, strikePower);
    if (millis() >= (triggerTime + strikeDuration))
      malletState = COAST;
    break;

  case COAST:
    ledcWrite(channel, coastPower);
    if (millis() >= (triggerTime + strikeDuration + coastDuration))
      malletState = REBOUND;
    break;

  case REBOUND:
    ledcWrite(channel, reboundPower);
    if (millis() >= (triggerTime + strikeDuration + coastDuration + reboundDuration))
      malletState = IDLESTATE;
    break;

  default:
    malletState = IDLESTATE;
    break;
  }
}

void Mallet::triggerMallet()
{
  malletState = TRIGGER;
}

bool Mallet::isIdle()
{
  if (malletState == IDLESTATE)
    return true;
  else
    return false;
}

void Mallet::setDelay(unsigned long lagTime)
{
  lag = lagTime;
}

//add queued strike to the buffer for future trigger with nonblocking delay
void Mallet::delayedTrigger(unsigned long delayTime)
{
  
  if (queuedStrikeCount >= StrikeQueueDepth) return; //too many queued, ignore this one.

  bool done = false;
  for(uint16_t k = 0; k < StrikeQueueDepth; k++){   //dont overwrite already occupied array index, find unoccupied one.
    if(!done){
      if(delayedStrikeTriggerTimes[k] == 0){                    
        delayedStrikeTriggerTimes[k] = millis() + delayTime;
        done = true;
        queuedStrikeCount++;
      }
    }
  }
}

//check if any queued strikes should be triggered. do so, and remove from buffer
void Mallet::handleQueuedStrikes()
{
  for (uint16_t i = 0; i < StrikeQueueDepth; i++)
  {
    if (delayedStrikeTriggerTimes[i] > 0)
    {
      if (millis() >= delayedStrikeTriggerTimes[i])
      {
        malletState = TRIGGER;
        delayedStrikeTriggerTimes[i] = 0;
        queuedStrikeCount -= 1;
      }
    }
  }
}

void Mallet::setMidiPitch(int pitch)
{
  MIDInote = pitch;
}

int Mallet::getMidiPitch()
{
  return MIDInote;
}

unsigned long Mallet::getDelay(){
  return lag;
}