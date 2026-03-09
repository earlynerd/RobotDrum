#include "Calibration.h"

#include <EEPROM.h>

namespace
{
constexpr uint32_t kCalibrationMagic = 0x52444232; // "RDB2"
constexpr uint16_t kCalibrationVersion = 3;

struct CalibrationHeader
{
  uint32_t magic;
  uint16_t version;
  uint16_t malletCount;
  uint32_t writeCount;
};

struct CalibrationEntry
{
  int16_t midiPitch;
  uint16_t softPower;
  uint16_t hardPower;
  uint16_t softLagMs;
  uint16_t hardLagMs;
  uint32_t softImpact;
  uint32_t hardImpact;
  uint8_t valid;
  uint8_t strikePct;       // rebound tuning: strike duration as % of lag (0 = default)
  uint16_t reboundPeakPwr; // rebound tuning: peak brake PWM (0 = default)
};

constexpr size_t kCalibrationHeaderAddress = 0;
constexpr size_t kCalibrationEntriesAddress = sizeof(CalibrationHeader);
constexpr size_t kEepromBytes = 512;

Mallet::CalibrationModel buildDefaultModel()
{
  return {
    460,
    1023,
    420,
    300,
    1,
    2,
    false,
    0,
    0,
  };
}

CalibrationEntry buildCalibrationEntry(const Mallet &mallet)
{
  const Mallet::CalibrationModel model = mallet.getCalibration();

  CalibrationEntry entry = {};
  entry.midiPitch = static_cast<int16_t>(mallet.getMidiPitch());
  entry.softPower = model.softPower;
  entry.hardPower = model.hardPower;
  entry.softLagMs = model.softLagMs;
  entry.hardLagMs = model.hardLagMs;
  entry.softImpact = model.softImpact;
  entry.hardImpact = model.hardImpact;
  entry.valid = model.valid ? 1 : 0;
  entry.strikePct = model.strikePct;
  entry.reboundPeakPwr = model.reboundPeakPwr;
  return entry;
}

void applyCalibrationEntry(Mallet &mallet, const CalibrationEntry &entry)
{
  if (entry.valid != 0)
  {
    Mallet::CalibrationModel model = {};
    model.softPower = entry.softPower;
    model.hardPower = entry.hardPower;
    model.softLagMs = entry.softLagMs;
    model.hardLagMs = entry.hardLagMs;
    model.softImpact = entry.softImpact;
    model.hardImpact = entry.hardImpact;
    model.valid = true;
    model.strikePct = entry.strikePct;
    model.reboundPeakPwr = entry.reboundPeakPwr;
    mallet.setCalibration(model);
  }
  else
  {
    mallet.setCalibration(buildDefaultModel());
  }

  if (entry.midiPitch >= 0 && entry.midiPitch <= 127)
  {
    mallet.setMidiPitch(entry.midiPitch);
  }
}

} // namespace

namespace Calibration
{

void setDefaults(Mallet &mallet)
{
  mallet.setCalibration(buildDefaultModel());
}

void initializeDefaults(Mallet *mallets, size_t count)
{
  for (size_t i = 0; i < count; ++i)
  {
    setDefaults(mallets[i]);
  }
}

unsigned long defaultLagFromModel(const Mallet::CalibrationModel &model)
{
  return static_cast<unsigned long>((static_cast<uint32_t>(model.softLagMs) +
                                     static_cast<uint32_t>(model.hardLagMs)) /
                                    2UL);
}

bool saveToEeprom(const Mallet *mallets, size_t count, Stream &out)
{
  const size_t requiredStorageBytes = sizeof(CalibrationHeader) + (sizeof(CalibrationEntry) * count);
  if (requiredStorageBytes > kEepromBytes)
  {
    out.println("Calibration save failed: storage layout exceeds EEPROM size");
    return false;
  }

  CalibrationHeader header = {};
  EEPROM.get(kCalibrationHeaderAddress, header);

  if (header.magic != kCalibrationMagic || header.version != kCalibrationVersion || header.malletCount != count)
  {
    header.writeCount = 0;
  }

  header.magic = kCalibrationMagic;
  header.version = kCalibrationVersion;
  header.malletCount = static_cast<uint16_t>(count);
  header.writeCount += 1;

  EEPROM.put(kCalibrationHeaderAddress, header);
  for (size_t i = 0; i < count; ++i)
  {
    const size_t address = kCalibrationEntriesAddress + (i * sizeof(CalibrationEntry));
    CalibrationEntry entry = buildCalibrationEntry(mallets[i]);
    EEPROM.put(address, entry);
  }

  const bool committed = EEPROM.commit();
  out.print("Saved calibration, writeCount=");
  out.print(header.writeCount);
  out.print(", status=");
  out.println(committed ? "ok" : "failed");
  return committed;
}

bool loadFromEeprom(Mallet *mallets, size_t count, Stream &out)
{
  CalibrationHeader header = {};
  EEPROM.get(kCalibrationHeaderAddress, header);

  if (header.magic != kCalibrationMagic || header.version != kCalibrationVersion || header.malletCount != count)
  {
    return false;
  }

  for (size_t i = 0; i < count; ++i)
  {
    CalibrationEntry entry = {};
    const size_t address = kCalibrationEntriesAddress + (i * sizeof(CalibrationEntry));
    EEPROM.get(address, entry);
    applyCalibrationEntry(mallets[i], entry);
  }

  out.print("Loaded calibration from EEPROM writes: ");
  out.println(header.writeCount);
  return true;
}

void printStatus(const Mallet *mallets, size_t count, Stream &out)
{
  out.println("Mallet status:");
  for (size_t i = 0; i < count; ++i)
  {
    const Mallet::CalibrationModel model = mallets[i].getCalibration();

    out.print("  #");
    out.print(i);
    out.print(" pitch=");
    out.print(mallets[i].getMidiPitch());
    out.print(" lagSoft=");
    out.print(model.softLagMs);
    out.print(" lagHard=");
    out.print(model.hardLagMs);
    out.print(" pwrSoft=");
    out.print(model.softPower);
    out.print(" pwrHard=");
    out.print(model.hardPower);
    out.print(" impactSoft=");
    out.print(model.softImpact);
    out.print(" impactHard=");
    out.print(model.hardImpact);
    out.print(" strike=");
    out.print(model.strikePct);
    out.print("% rebPwr=");
    out.print(model.reboundPeakPwr);
    out.print(" valid=");
    out.print(model.valid ? "yes" : "no");

    // Print minimum cycle time for composition reference.
    // Computed for hard velocity (shortest lag = tightest retrigger constraint).
    if (model.valid)
    {
      const float lagF = static_cast<float>(model.hardLagMs);
      const float strikeMul = (model.strikePct > 0)
        ? (static_cast<float>(model.strikePct) / 100.0f) : 1.50f;
      const int strikeDur = constrain(static_cast<int>(lagF * strikeMul), 60, 600);
      const int reboundDur = constrain(static_cast<int>(lagF * 1.40f), 30, 300);
      const unsigned long cycleMs = static_cast<unsigned long>(strikeDur + reboundDur)
                                    + mallets[i].getRetriggerGap();
      out.print(" cycleMs=");
      out.print(cycleMs);
    }
    out.println();
  }
}

} // namespace Calibration
