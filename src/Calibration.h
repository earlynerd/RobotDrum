#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "mallet.h"

namespace Calibration
{

void setDefaults(Mallet &mallet);
void initializeDefaults(Mallet *mallets, size_t count);
unsigned long defaultLagFromModel(const Mallet::CalibrationModel &model);
bool saveToEeprom(const Mallet *mallets, size_t count, Stream &out);
bool loadFromEeprom(Mallet *mallets, size_t count, Stream &out);
void printStatus(const Mallet *mallets, size_t count, Stream &out);

} // namespace Calibration

#endif
