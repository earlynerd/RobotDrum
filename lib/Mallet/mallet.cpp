#include "Mallet.h"

namespace
{
bool timeReached(unsigned long now, unsigned long target)
{
  return static_cast<long>(now - target) >= 0;
}

bool timeBefore(unsigned long a, unsigned long b)
{
  return static_cast<long>(a - b) < 0;
}
}

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
    {
      lastCycleEndMillis = millis();
      malletState = IDLESTATE;
    }
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

void Mallet::setRetriggerGap(unsigned long gapMs)
{
  minRetriggerGapMs = gapMs;
}

void Mallet::abortAndClearQueue()
{
  for (uint16_t i = 0; i < StrikeQueueDepth; ++i)
  {
    delayedStrikeTriggerTimes[i] = 0;
    delayedStrikeHasProfile[i] = false;
  }
  queuedStrikeCount = 0;
  malletState = IDLESTATE;
  triggerTime = millis();
  lastCycleEndMillis = triggerTime;
  ledcWrite(channel, 0);
}

//add queued strike to the buffer for future trigger with nonblocking delay
void Mallet::delayedTrigger(unsigned long delayTime)
{
  StrikeProfile profile = {
    strikePower,
    strikeDuration,
    coastPower,
    coastDuration,
    reboundPower,
    reboundDuration
  };
  delayedTrigger(delayTime, profile);
}

void Mallet::delayedTrigger(unsigned long delayTime, const StrikeProfile &profile)
{
  if (queuedStrikeCount >= StrikeQueueDepth) return; //too many queued, ignore this one.

  for (uint16_t k = 0; k < StrikeQueueDepth; k++)
  {
    if (delayedStrikeTriggerTimes[k] == 0)
    {
      delayedStrikeTriggerTimes[k] = millis() + delayTime;
      delayedStrikeProfiles[k] = profile;
      delayedStrikeHasProfile[k] = true;
      queuedStrikeCount++;
      return;
    }
  }
}

//check if any queued strikes should be triggered. do so, and remove from buffer
void Mallet::handleQueuedStrikes()
{
  if (queuedStrikeCount == 0)
  {
    return;
  }

  if (malletState != IDLESTATE)
  {
    return;
  }

  const unsigned long now = millis();
  if ((now - lastCycleEndMillis) < minRetriggerGapMs)
  {
    return;
  }

  int candidate = -1;
  unsigned long candidateTime = 0;

  for (uint16_t i = 0; i < StrikeQueueDepth; i++)
  {
    const unsigned long triggerTimeMs = delayedStrikeTriggerTimes[i];
    if (triggerTimeMs > 0 && timeReached(now, triggerTimeMs))
    {
      if (candidate < 0 || timeBefore(triggerTimeMs, candidateTime))
      {
        candidate = static_cast<int>(i);
        candidateTime = triggerTimeMs;
      }
    }
  }

  if (candidate < 0)
  {
    return;
  }

  const uint16_t selected = static_cast<uint16_t>(candidate);
  if (delayedStrikeHasProfile[selected])
  {
    strikePower = delayedStrikeProfiles[selected].strikePower;
    strikeDuration = delayedStrikeProfiles[selected].strikeDuration;
    coastPower = delayedStrikeProfiles[selected].coastPower;
    coastDuration = delayedStrikeProfiles[selected].coastDuration;
    reboundPower = delayedStrikeProfiles[selected].reboundPower;
    reboundDuration = delayedStrikeProfiles[selected].reboundDuration;
  }

  malletState = TRIGGER;
  delayedStrikeTriggerTimes[selected] = 0;
  delayedStrikeHasProfile[selected] = false;
  if (queuedStrikeCount > 0)
  {
    queuedStrikeCount -= 1;
  }
}

void Mallet::setMidiPitch(int pitch)
{
  MIDInote = pitch;
}

int Mallet::getMidiPitch() const
{
  return MIDInote;
}

unsigned long Mallet::getDelay() const{
  return lag;
}

unsigned long Mallet::getRetriggerGap() const
{
  return minRetriggerGapMs;
}

void Mallet::setCalibration(const CalibrationModel &model)
{
  calibration = model;
}

const Mallet::CalibrationModel &Mallet::getCalibration() const
{
  return calibration;
}
