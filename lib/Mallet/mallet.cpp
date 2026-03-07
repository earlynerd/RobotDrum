#include "Mallet.h"

namespace
{
// Unsigned millis() comparisons that handle rollover correctly.
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
}

void Mallet::begin()
{
  ledcAttachPin(malletPin, channel);
  ledcSetup(channel, 18000, 10);  // 18 kHz PWM, 10-bit resolution
}

// Build a simple 3-segment profile from the legacy member variables.
// This is the compatibility path for calibration code that sets
// strikePower/Duration etc. directly rather than building a full
// shaped profile.
Mallet::StrikeProfile Mallet::buildProfileFromMembers() const
{
  StrikeProfile profile = {};
  profile.count = 0;

  if (strikeDuration > 0)
  {
    profile.segments[profile.count++] = {
      static_cast<uint16_t>(constrain(strikePower, 0, 1023)),
      static_cast<uint16_t>(strikeDuration)
    };
  }

  if (coastDuration > 0)
  {
    profile.segments[profile.count++] = {
      static_cast<uint16_t>(constrain(coastPower, 0, 1023)),
      static_cast<uint16_t>(coastDuration)
    };
  }

  if (reboundDuration > 0)
  {
    profile.segments[profile.count++] = {
      static_cast<uint16_t>(constrain(reboundPower, 0, 1023)),
      static_cast<uint16_t>(reboundDuration)
    };
  }

  return profile;
}

// Core state machine — called every loop() iteration for each mallet.
// Walks through the active profile's segments sequentially, setting PWM
// power for each segment's duration, then returns to idle.
void Mallet::updateMallet()
{
  handleQueuedStrikes();

  switch (malletState)
  {
  case IDLESTATE:
    break;

  case TRIGGER:
    triggerTime = millis();
    activeSegment = 0;
    if (activeProfile.count > 0)
    {
      segmentEndMillis = triggerTime + activeProfile.segments[0].durationMs;
      malletState = ACTIVE;
    }
    else
    {
      ledcWrite(channel, 0);
      lastCycleEndMillis = millis();
      malletState = IDLESTATE;
    }
    break;

  case ACTIVE:
    if (activeSegment >= activeProfile.count)
    {
      ledcWrite(channel, 0);
      lastCycleEndMillis = millis();
      malletState = IDLESTATE;
      break;
    }
    ledcWrite(channel, activeProfile.segments[activeSegment].power);
    if (millis() >= segmentEndMillis)
    {
      activeSegment++;
      if (activeSegment < activeProfile.count)
      {
        // Accumulate rather than re-read millis() so segment durations
        // don't drift from loop latency.
        segmentEndMillis += activeProfile.segments[activeSegment].durationMs;
      }
    }
    break;

  default:
    ledcWrite(channel, 0);
    malletState = IDLESTATE;
    break;
  }
}

// Immediate trigger using legacy member params (used by calibration).
void Mallet::triggerMallet()
{
  activeProfile = buildProfileFromMembers();
  activeSegment = 0;
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
  portENTER_CRITICAL(&queueMux);
  for (uint16_t i = 0; i < StrikeQueueDepth; ++i)
  {
    delayedStrikeTriggerTimes[i] = 0;
    delayedStrikeHasProfile[i] = false;
  }
  queuedStrikeCount = 0;
  portEXIT_CRITICAL(&queueMux);
  malletState = IDLESTATE;
  triggerTime = millis();
  lastCycleEndMillis = triggerTime;
  ledcWrite(channel, 0);
}

// Queue a strike for future execution using the legacy member params.
void Mallet::delayedTrigger(unsigned long delayTime)
{
  delayedTrigger(delayTime, buildProfileFromMembers());
}

// Queue a strike with a pre-built shaped profile. The profile is stored
// in a ring buffer and dequeued by handleQueuedStrikes() when its
// scheduled time arrives. Called from the BLE MIDI callback (Core 0)
// with the spinlock protecting concurrent access.
void Mallet::delayedTrigger(unsigned long delayTime, const StrikeProfile &profile)
{
  portENTER_CRITICAL(&queueMux);
  if (queuedStrikeCount >= StrikeQueueDepth)
  {
    portEXIT_CRITICAL(&queueMux);
    return;
  }

  for (uint16_t k = 0; k < StrikeQueueDepth; k++)
  {
    if (delayedStrikeTriggerTimes[k] == 0)
    {
      delayedStrikeTriggerTimes[k] = millis() + delayTime;
      delayedStrikeProfiles[k] = profile;
      delayedStrikeHasProfile[k] = true;
      queuedStrikeCount++;
      portEXIT_CRITICAL(&queueMux);
      return;
    }
  }
  portEXIT_CRITICAL(&queueMux);
}

// Dequeue the oldest ready strike and begin executing it. Picks the
// earliest-scheduled entry that has reached its trigger time. Enforces
// a minimum gap between strike cycles to prevent retriggering before
// the mallet has settled.
void Mallet::handleQueuedStrikes()
{
  portENTER_CRITICAL(&queueMux);
  if (queuedStrikeCount == 0)
  {
    portEXIT_CRITICAL(&queueMux);
    return;
  }

  if (malletState != IDLESTATE)
  {
    portEXIT_CRITICAL(&queueMux);
    return;
  }

  const unsigned long now = millis();
  if ((now - lastCycleEndMillis) < minRetriggerGapMs)
  {
    portEXIT_CRITICAL(&queueMux);
    return;
  }

  // Find the oldest queued strike whose time has arrived.
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
    portEXIT_CRITICAL(&queueMux);
    return;
  }

  const uint16_t selected = static_cast<uint16_t>(candidate);
  if (delayedStrikeHasProfile[selected])
  {
    activeProfile = delayedStrikeProfiles[selected];
  }
  else
  {
    activeProfile = buildProfileFromMembers();
  }

  activeSegment = 0;
  malletState = TRIGGER;
  delayedStrikeTriggerTimes[selected] = 0;
  delayedStrikeHasProfile[selected] = false;
  if (queuedStrikeCount > 0)
  {
    queuedStrikeCount -= 1;
  }
  portEXIT_CRITICAL(&queueMux);
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
