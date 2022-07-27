#pragma once

// #define __ACCEL

#ifdef __ACCEL

void initADXL345();
float getTilt();

extern bool accelInitied;

// tilt offset comes from the difference of the mounted tilt sensor and the telescope axis
#define TILT_OFFSET (-100.01 + 90)
#define TILT() dRange(TILT_OFFSET - getTilt(), 180., -180.)

#endif // __ACCEL