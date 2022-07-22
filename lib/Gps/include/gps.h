#pragma once

#define __GPS

#include <TimeLib.h>

#ifdef __GPS

void initGPS();
void tickGPS();

float getGpsLat();
float getGpsLon();
time_t getGpsDateTime();
bool isGpsFix();
int getGpsSatellites();

#endif // __GPS