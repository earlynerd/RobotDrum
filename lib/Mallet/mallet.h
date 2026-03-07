#ifndef __MALLET_H
#define __MALLET_H

#include <Arduino.h>
#include <freertos/portmacro.h>

#define StrikeQueueDepth    10

// Drives a single mallet actuator through a shaped PWM profile.
//
// Each mallet is a brushed DC motor + clockspring. The motor can only push
// the mallet toward the drum (single N-FET, no H-bridge). The clockspring
// provides all return force. The drive circuit is single-quadrant: we can
// apply forward torque (0-1023 PWM) or coast (0 PWM). No reverse torque
// or active braking is available.
//
// A strike cycle is defined as a StrikeProfile: a sequence of up to 8
// (power, duration) segments executed back-to-back. A typical shaped
// profile looks like:
//
//   PWM
//    |  /--\______________
//    | /                  |          <- strike: ramp up then hold
//    |/                   |
//    |                    |__        <- coast: motor off around impact
//    |                       |____   <- rebound: tapered brake as spring
//    |                            |     returns mallet toward rest
//    +-----------------------------> time
//
// The ramp softens the motor turn-on transient (reduces audible click).
// The tapered rebound applies forward torque while the mallet is moving
// away from the drum under spring force, which acts as a brake (opposing
// direction of motion). The taper is front-loaded: strong braking early
// when the mallet is fastest, then fading so the spring can gently carry
// the mallet home without overshoot or bumpstop impact.
//
// For MIDI playback, buildStrikeForVelocity() (in RobotDrum.cpp) generates
// shaped profiles from calibration data. For calibration probes, the legacy
// member variables (strikePower, coastPower, etc.) are set directly and
// buildProfileFromMembers() converts them into a simple 3-segment profile.
class Mallet{
  public:
  // Two-point calibration model measured during automated calibration.
  // "soft" and "hard" represent the endpoints of the velocity range.
  // lagMs = time from energize to drum contact at that power level.
  // impact = peak mic amplitude from the strike (proxy for loudness).
  struct CalibrationModel
  {
    uint16_t softPower;
    uint16_t hardPower;
    uint16_t softLagMs;
    uint16_t hardLagMs;
    uint32_t softImpact;
    uint32_t hardImpact;
    bool valid;
  };

  struct StrikeSegment
  {
    uint16_t power;       // PWM duty 0-1023 (10-bit)
    uint16_t durationMs;
  };

  static constexpr uint8_t kMaxStrikeSegments = 8;

  // A complete drive waveform: segments are executed sequentially.
  // count=0 means no-op (mallet stays idle).
  struct StrikeProfile
  {
    StrikeSegment segments[kMaxStrikeSegments];
    uint8_t count;
  };

  Mallet(int pin, int MIDIpitch, int malletChannel, int strikePWM, int strikeTime, int coastPWM, int coastTime, int reboundPWM, int reboundTime);
  void begin();
  int channel;
  int MIDInote;

  // Legacy per-phase parameters. Used by calibration code which sets these
  // directly, then calls triggerMallet(). buildProfileFromMembers() converts
  // them into a 3-segment profile (strike/coast/rebound at constant power).
  int strikePower;
  int strikeDuration;
  int coastDuration;
  int coastPower;
  int reboundDuration;
  int reboundPower;

  void updateMallet();      // call every loop iteration to advance the profile
  void triggerMallet();     // start immediately using legacy member params
  void delayedTrigger(unsigned long delayTime);
  void delayedTrigger(unsigned long delayTime, const StrikeProfile &profile);
  bool isIdle();
  void setDelay(unsigned long lagTime);
  void setRetriggerGap(unsigned long gapMs);
  void abortAndClearQueue();
  int getMidiPitch() const;
  void setMidiPitch(int pitch);
  unsigned long getDelay() const;
  unsigned long getRetriggerGap() const;
  void setCalibration(const CalibrationModel &model);
  const CalibrationModel &getCalibration() const;

  // Convert legacy member variables into a flat 3-segment profile.
  // Used as the bridge between old-style calibration code (which sets
  // strikePower/Duration etc. directly) and the segment-based executor.
  StrikeProfile buildProfileFromMembers() const;

  // IDLE: waiting for trigger. TRIGGER: latch start time, begin first
  // segment. ACTIVE: executing segments sequentially until done.
  enum State {
  IDLESTATE=0,
  TRIGGER,
  ACTIVE
};


  private:
  unsigned long lag;
  unsigned long delayedStrikeTriggerTimes[StrikeQueueDepth] = {};
  StrikeProfile delayedStrikeProfiles[StrikeQueueDepth] = {};
  bool delayedStrikeHasProfile[StrikeQueueDepth] = {};
  uint16_t queuedStrikeCount = 0;
  void handleQueuedStrikes();
  unsigned long minRetriggerGapMs = 0;
  unsigned long lastCycleEndMillis = 0;
  CalibrationModel calibration = {460, 1023, 420, 300, 1, 2, false};

  State malletState = IDLESTATE;
  StrikeProfile activeProfile = {};   // profile currently being executed
  uint8_t activeSegment = 0;          // index into activeProfile.segments
  unsigned long segmentEndMillis = 0;  // millis() when current segment expires

  unsigned long triggerTime;
  int malletPin;
  portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;
};
#endif
