#pragma once

#include <math.h>
#include <TimeLib.h>

/**
 * Astronomy calculations
 *
 * Holds local position (GPS Long/Lat, time)
 *
 * sets and reads current direction in RADec / AltAzi
 */
class MyAstro
{
protected:
    double inRange24(double d);
    double inRange360(double d);
    double inRange2PI(double d);
    double deg2rad(double n);
    double rad2deg(double n);

public:
    bool setLatLong(double latitude, double longitude);
    bool applyAltAz(double alt, double azi);
    bool applyRAdec(double ra, double dec);

    float getLocalSiderealTime();

    double getRAdec(void);
    double getDeclinationDec(void);
    double getAzimuth(void);
    double getAltitude(void);
    double getLatDec(void);
    double getLongDec(void);

    /**
     * @brief Get the Current Time object
     *
     * @return unsigned long in second
     */
    unsigned long long getCurrentTime();
    /**
     * @brief Set the Current Time object
     *
     * @param currentTime in second
     */
    void setCurrentTime(unsigned long long currentTime);

private:
    double decLat;
    double radLat;
    double decLong;
    double radLong;

    double cosAlt;
    double sinAlt;
    double cosAz;
    double sinAz;

    double sinDec;
    double cosDec;
    double sinLat;
    double cosLat;

    double RAdec;
    double RArad;
    double sinRA;
    double cosRA;
    double DeclinationDec;
    // double RightAscension;
    // double Declination;
    double DeclinationRad;
    double AzDec;
    double AzRad;
    double AltDec;
    double AltRad;

    const double F2PI = 2.0 * M_PI;
    const double FPI = M_PI;
    const double FPIdiv2 = M_PI_2;
    const double FminusPIdiv2 = -M_PI_2;
    const double FPIdiv4 = M_PI_4;

    unsigned long long timeBase = 0; // origin time, in second
    double lastTime = 0.0;           // last time RA/Dec was set. If local time is different from this, Alt/Azi must be recalculated (note EPSILON_TIME)
};