#pragma once

#include <EEPROM.h>

#define EEPROM_LATLON_ADDRESS (sizeof(float) * 10)

inline void saveLatLonToEEPROM(float lat, float lon)
{
    EEPROM.put(EEPROM_LATLON_ADDRESS, lat);
    EEPROM.put(EEPROM_LATLON_ADDRESS + sizeof(float), lon);
}

inline float readLatFromEEPROM()
{
    float f;
    EEPROM.get(EEPROM_LATLON_ADDRESS, f);
    return f;
}

inline float readLonFromEEPROM()
{
    float f;
    EEPROM.get(EEPROM_LATLON_ADDRESS + sizeof(float), f);
    return f;
}

