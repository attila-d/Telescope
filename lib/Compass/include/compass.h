#pragma once

#include <MPU9250.h>

void initMPU9250();
void tickMPU9250();

float getNorth();

// #define __MAGNETO

#ifdef __MAGNETO

extern bool compassInited;

// north offset comes from the difference of the mounted magnetometer north and chassis north axis
#define NORTH_OFFSET 92
#define NORTH() dRange(360 - (getNorth() - NORTH_OFFSET), -180., 180.)

MPU9250 &getMPU9250();

void beginMagCalibration();
void tickMagCalibration();
void endMagCalibration();

#endif // __MAGNETO