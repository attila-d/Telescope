#include "astro.h"

#include "Arduino.h"
#include "math.h"

time_t MyAstro::getCurrentTime()
{
    unsigned long l = (millis() / 1000) - timeBase;
    // DEBUG("get current time:");
    // DEBUG2LN(l, DEC);
    return l;
}

void MyAstro::setCurrentTime(time_t currentTime)
{
    // DEBUG("set current time:");
    // DEBUG2LN(currentTime, DEC);
    timeBase = millis() / 1000 - currentTime;
}

double MyAstro::inRange24(double d)
{
    while (d < 0.)
    {
        d += 24.;
    }
    while (d >= 24.)
    {
        d -= 24.;
    }
    return d;
}

double MyAstro::inRange360(double d)
{
    while (d < 0.)
    {
        d += 360.;
    }
    while (d >= 360.)
    {
        d -= 360.;
    }
    return d;
}

double MyAstro::inRange2PI(double d)
{
    while (d < 0.)
    {
        d += F2PI;
    }
    while (d >= F2PI)
    {
        d -= F2PI;
    }
    return d;
}

double MyAstro::deg2rad(double n)
{
    return n * 1.745329252e-2;
}

double MyAstro::rad2deg(double n)
{
    return n * 5.729577951e1;
}

double MyAstro::getRAdec()
{
    RAdec = rad2deg(RArad) / 15.;
    return RAdec;
}

double MyAstro::getDeclinationDec()
{
    DeclinationDec = rad2deg(DeclinationRad);
    return DeclinationDec;
}

double MyAstro::getAltitude()
{
    AltDec = rad2deg(AltRad);
    return AltDec;
}

double MyAstro::getAzimuth()
{
    AzDec = rad2deg(AzRad);
    return AzDec;
}

double MyAstro::getLatDec()
{
    return decLat;
}

double MyAstro::getLongDec()
{
    return decLong;
}

double MyAstro::getLocalSiderealTime()
{
    time_t obstime = getCurrentTime();
    double d, t, GMST_s, LMST_s;

    d = (obstime / 86400.0) + 2440587.5 - 2451545.0;
    t = d / 36525;

    // GMST_s = 24110.54841 + 8640184.812866 * t + 0.093104 * pow(t, 2) - 0.0000062 * pow(t, 3);
    GMST_s = 24110.54841 + 8640184.812866 * t + 0.093104 * t * t - 0.0000062 * t * t * t;
    /* convert from UT1=0 */
    GMST_s += obstime;
    GMST_s = GMST_s - 86400.0 * floor(GMST_s / 86400.0);

    /* adjust to LMST */
    LMST_s = GMST_s + 3600 * decLong / 15.;

    if (LMST_s <= 0)
    { /* LMST is between 0 and 24h */
        LMST_s += 86400.0;
    }

    // DEBUG3(" sid2 ", LMST_s / 3600., 6);
    return LMST_s / 3600.;
}

bool MyAstro::setLatLong(double latitude, double longitude)
{
    // Input latitude and longitude are in decimal degrees
    // save these, and also save values in radians
    if (decLat == latitude && decLong == longitude)
        return true; // Already did it
    decLat = latitude;
    radLat = deg2rad(decLat);
    decLong = longitude;
    radLong = deg2rad(decLong);
    cosLat = cos(radLat);
    sinLat = sin(radLat);
    // risetDone = false;
    return true;
}

/**
 * sets alt and azimuth for astro calculations. It also updates RA and Dec
 */
bool MyAstro::applyAltAz(double Altitude, double Azimuth)
{
    if (AltDec == Altitude && AzDec == Azimuth)
        return true; // Already done
    AltDec = Altitude;
    AltRad = deg2rad(AltDec);
    sinAlt = sin(AltRad);
    cosAlt = cos(AltRad);
    AzDec = Azimuth;
    AzRad = deg2rad(AzDec);
    sinAz = sin(AzRad);
    cosAz = cos(AzRad);

    // doAltAzi2RaDec
    sinDec = (sinAlt * sinLat) + (cosAlt * cosLat * cosAz);
    DeclinationRad = asin(sinDec);
    cosDec = cos(DeclinationRad);
    double b = cosLat * cosDec;
    if (b < 1e-10)
        b = 1e-10;
    double cosHA = (sinAlt - (sinLat * sinDec)) / b;
    double HArad = acos(cosHA);
    if (sinAz > 0)
        HArad = F2PI - HArad;
    double HAdec = rad2deg(HArad) / 15.0;
    double RAdec = inRange24(getLocalSiderealTime() - HAdec);
    RArad = deg2rad(RAdec * 15.);
    sinRA = sin(RArad);
    cosRA = cos(RArad);
    cosDec = cos(DeclinationRad);
    return true;
}

/**
 * Sets RA and Dec, and updates and calculates Alt/Azimuth
 */
bool MyAstro::applyRAdec(double RightAscension, double Declination)
{
    if (RAdec == RightAscension && DeclinationDec == Declination)
        return true; // Already done
    RAdec = RightAscension;
    RArad = deg2rad(RAdec) * 15.;
    sinRA = sin(RArad);
    cosRA = cos(RArad);
    DeclinationDec = Declination;
    DeclinationRad = deg2rad(DeclinationDec);
    sinDec = sin(DeclinationRad);
    cosDec = cos(DeclinationRad);

    double HAdec = inRange24(getLocalSiderealTime() - RAdec);
    double HArad = deg2rad(HAdec * 15.0);
    double cosHA = cos(HArad);
    double sinHA = sin(HArad);
    sinAlt = (sinDec * sinLat) + (cosDec * cosLat * cosHA);
    AltRad = asin(sinAlt);
    cosAlt = cos(AltRad);
    double b = cosLat * cosAlt;
    if (b < 1e-10)
        b = 1e-10;
    cosAz = (sinDec - (sinLat * sinAlt)) / b;
    AzRad = acos(cosAz);
    if (sinHA > 0)
        AzRad = F2PI - AzRad;
    sinAz = sin(AzRad);
    return true;
}
