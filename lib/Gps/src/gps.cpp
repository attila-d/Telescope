#include <Arduino.h>

#include "gps.h"
#include "debug.h"

#include <TimeLib.h>
// #include <SoftwareSerial.h>
#include <TinyGPS.h>

/**
 *
 $GPGGA,181908.00,3404.7041778,N,07044.3966270,W,4,13,1.00,495.144,M,29.200,M,0.10,0000*40
All NMEA messages start with the $ character, and each data field is separated by a comma.
GP represent that it is a GPS position (GL would denote GLONASS).
181908.00 is the time stamp: UTC time in hours, minutes and seconds.
3404.7041778 is the latitude in the DDMM.MMMMM format. Decimal places are variable.
N denotes north latitude.
07044.3966270 is the longitude in the DDDMM.MMMMM format. Decimal places are variable.
W denotes west longitude.
4 denotes the Quality Indicator:
1 = Uncorrected coordinate
2 = Differentially correct coordinate (e.g., WAAS, DGPS)
4 = RTK Fix coordinate (centimeter precision)
5 = RTK Float (decimeter precision.
13 denotes number of satellites used in the coordinate.
1.0 denotes the HDOP (horizontal dilution of precision).
495.144 denotes altitude of the antenna.
M denotes units of altitude (eg. Meters or Feet)
29.200 denotes the geoidal separation (subtract this from the altitude of the antenna to arrive at the Height Above Ellipsoid (HAE).
M denotes the units used by the geoidal separation.
1.0 denotes the age of the correction (if any).
0000 denotes the correction station ID (if any).
*40 denotes the checksum.
*/

TinyGPS myTinyGPS;

void initGPS()
{
    Serial2.begin(9600); // NMEA PS
}

bool GPSfix = false;

void tickGPS()
{
    while (Serial2.available() > 0)
    {
        char c = Serial2.read();
        // DEBUGW(c);
        if (myTinyGPS.encode(c))
        { // ready, actual data is available
            GPSfix = true;
#ifdef _DEBUG
            long lon, lat;
            unsigned long age;
            unsigned long dt, tm;
            myTinyGPS.get_position(&lat, &lon, &age);
            myTinyGPS.get_datetime(&dt, &tm, &age);
            DEBUG("GPS:[");
            DEBUG2(lat, 6);
            DEBUG("/");
            DEBUG2(lon, 6);
            DEBUG(" time");
            DEBUG2(dt, DEC);
            DEBUG("/");
            DEBUG2(tm, DEC);
            DEBUG("], sat=");
            DEBUG2LN(myTinyGPS.satellites(), DEC);
#endif
        }
    }
}

float getGpsLat()
{
    float lat;
    myTinyGPS.f_get_position(&lat, NULL, NULL);
    return lat;
}

float getGpsLon()
{
    float lon;
    myTinyGPS.f_get_position(NULL, &lon, NULL);
    return lon;
}

/**
 * @brief Get the Gps Date Time object
 *
 * @return time_t in seconds
 */
time_t getGpsDateTime()
{
    int year;
    byte month, day, hour, minute, second, hundredths;
    myTinyGPS.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, NULL);
    tmElements_t tmSet;
    tmSet.Year = year;
    tmSet.Month = month;
    tmSet.Day = day;
    tmSet.Hour = hour;
    tmSet.Minute = minute;
    tmSet.Second = second;
    return makeTime(tmSet);
}

bool isGpsFix()
{
    return GPSfix;
}

int getGpsSatellites()
{
    return (int)myTinyGPS.satellites();
}