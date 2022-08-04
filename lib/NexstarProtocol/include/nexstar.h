#pragma once

#include <Arduino.h>
#include <TimeLib.h>
//#include "telescopectrl.h"
#include "astro.h"
#include "util.h"
#include "debug.h"

class NexstarControl
{
public:
  int altSlew = 0;
  int aziSlew = 0;

  double altAuto = 0.;
  double aziAuto = 0.;

  MyAstro &astro;

public:
  NexstarControl(MyAstro &_astro) : astro(_astro)
  {
  }

  virtual boolean isGpsLinked() = 0;
  virtual boolean isGotoInProgress() = 0;
  virtual boolean isAligned() = 0;

  virtual void updateChassis() = 0;
  virtual void syncCurrentPosToRaDec(double ra, double dec) = 0;
  virtual int getTrackingMode() = 0;

  virtual void gotoAltAzi(double alt, double azi);
  virtual void gotoRaDec(double ra, double dec);
  virtual void cancelGoto() = 0;
};

/**
 * @brief takes TelescopeControl as argument, and a Comm device. Reads Comm and applies commands to TelescopeControl
 *
 */
class NexstarProtocol
{
public:
  uint8_t buffer[64];
  NexstarControl &telescope;
  Stream &comm;

public:
  int cnt = 0;

  boolean useDST = true;
  int GMToffset = +1; // time zone
private:
  bool respond()
  {
    comm.write('#');
    comm.flush();
    return true;
  }

  bool respond(uint8_t chr)
  {
    comm.write(chr);
    return respond();
  }

  bool respond(uint8_t chr1, uint8_t chr2)
  {
    comm.write(chr1);
    return respond(chr2);
  }

  bool respond(uint8_t chr1, uint8_t chr2, uint8_t chr3)
  {
    comm.write(chr1);
    return respond(chr2, chr3);
  }

  bool respond(uint8_t chr1, uint8_t chr2, uint8_t chr3, uint8_t chr4, uint8_t chr5, uint8_t chr6, uint8_t chr7, uint8_t chr8)
  {
    comm.write(chr1);
    comm.write(chr2);
    comm.write(chr3);
    comm.write(chr4);
    comm.write(chr5);
    return respond(chr6, chr7, chr8);
  }

  void respondHex(uint8_t byte)
  {
    int c = byte >> 4;
    c &= 0x0f;
    if (c < 10)
    {
      comm.write('0' + c);
    }
    else
    {
      comm.write('A' + c - 10);
    }
    c = byte & 0x0f;
    if (c < 10)
    {
      comm.write('0' + c);
    }
    else
    {
      comm.write('A' + c - 10);
    }
  }

  unsigned long readHex(uint8_t a, uint8_t b)
  {
    unsigned long res = 0;
    if (a >= '0' && a <= '9')
    {
      res = a - '0';
    }
    else if (a >= 'A' && a <= 'F')
    {
      res = a - 'A' + 10;
    }

    res <<= 4;

    if (b >= '0' && b <= '9')
    {
      res |= b - '0';
    }
    else if (b >= 'A' && b <= 'F')
    {
      res |= b - 'A' + 10;
    }

    return res;
  }

  double dMap(double x, double fromLow, double fromHigh, double toLow, double toHigh)
  {
    return toLow + ((x - fromLow) / (fromHigh - fromLow)) * (toHigh - toLow);
  }

  bool procGetPosition()
  {
    if (cnt == 1)
    {
      switch (buffer[0])
      {
      case 'E': // Get RA/DEC v1.2
        // “34AB,12CE#”
        DEBUG("GetPosition RA/DEC");
        DEBUGLN();
        {
          telescope.updateChassis();
          // unsigned long ra = map(telescope.getAlt(),-90.,90.,0.,65536.);
          // unsigned long dec = map(telescope.getAzimuth(), -180., 180., 0., 65536.);
          unsigned long ra = (telescope.astro.getRAdec() / 24.) * 65536.;
          unsigned long dec = (telescope.astro.getDeclinationDec() / 360.) * 65536.;
          respondHex(ra >> 8 & 0xff);
          respondHex(ra & 0xff);
          comm.write(',');
          respondHex(dec >> 8 & 0xff);
          respondHex(dec & 0xff);
          return respond();
        }
        break;
      case 'e': // Get precise RA/DEC, v1.6
        // DEBUG("GetPosition precise RA/DEC");
        // DEBUGLN();
        // “34AB0500,12CE0500#”
        {
          telescope.updateChassis();

          double dra = telescope.astro.getRAdec();
          double ddec = telescope.astro.getDeclinationDec();
          unsigned long ra = (dra / 24.) * (65536. * 256);
          unsigned long dec = (ddec / 360.) * (65536. * 256);
          // DEBUG2(ra, HEX);
          // DEBUG3("->",dec, HEX);
          // DEBUG3("; ",dra, 4);
          // DEBUG3(":",ddec, 4);
          // DEBUG("; Chassis CURR=");
          // DEBUG2(getAltCurrent(), DEC);
          // DEBUG(":");
          // DEBUG2(getAziCurrent(), DEC);
          // DEBUG("; Chassis TRG=");
          // DEBUG2(getAltTarget(), DEC);
          // DEBUG(":");
          // DEBUG2(getAziTarget(), DEC);
          // DEBUG("; TeleAlt/Azi=");
          // DEBUG2(getAstroAltitude(), 4);
          // DEBUG(":");
          // DEBUG2(getAstroAzimuth(), 4);
          // DEBUGLN();
          respondHex((ra >> 16) & 0xff); // only the upper 24 bits are in use
          respondHex((ra >> 8) & 0xff);
          respondHex((ra)&0xff);
          respondHex(0);
          comm.write(',');
          respondHex((dec >> 16) & 0xff);
          respondHex((dec >> 8) & 0xff);
          respondHex((dec)&0xff);
          respondHex(0);
          return respond();
        }
        break;
      case 'Z': // Get AZM-ALT v1.2
        DEBUG("GetPosition AZM/ALT");
        DEBUGLN();
        // “12AB,4000#”
        {
          // telescope.setAstroTime();
          telescope.updateChassis();
          long alt = map(telescope.astro.getAltitude(), -90, 90, 0, 65536);
          long azi = map(telescope.astro.getAzimuth(), -180, 180, 0, 65536);
          respondHex(alt >> 8 & 0xff);
          respondHex(alt & 0xff);
          comm.write(',');
          respondHex(azi >> 8 & 0xff);
          respondHex(azi & 0xff);
          return respond();
        }
        break;
      case 'z': // Get precise AZM-ALT v2.2
        DEBUG("GetPosition precise AZM/ALT");
        DEBUGLN();
        // “12AB0500,40000500#”
        {
          // telescope.setAstroTime();
          telescope.updateChassis();
          unsigned long alt = (unsigned long)((dMap(telescope.astro.getAltitude(), -90., 90., 0., 65536.) * 256.));
          unsigned long azi = (unsigned long)((dMap(telescope.astro.getAzimuth(), -180., 180., 0., 65536.) * 256.));
          respondHex((alt >> 16) & 0xff); // only the upper 24 bits are in use
          respondHex((alt >> 8) & 0xff);
          respondHex((alt)&0xff);
          respondHex(0);
          comm.write(',');
          respondHex((azi >> 16) & 0xff);
          respondHex((azi >> 8) & 0xff);
          respondHex((azi)&0xff);
          respondHex(0);
          return respond();
        }
        break;
      }
    }
    return false;
  }

  bool procGoto()
  {
    switch (buffer[0])
    {
    case 'R': // GOTO RA/DEC
      if (cnt == 10)
      {
        DEBUG("GoTo to RA/DEC:");
        // “R34AB,12CE”
        unsigned long ra = (readHex(buffer[1], buffer[2]) << 8) + readHex(buffer[3], buffer[4]);
        unsigned long dec = (readHex(buffer[6], buffer[7]) << 8) + readHex(buffer[8], buffer[9]);
        DEBUG2(ra, HEX);
        DEBUG(":");
        DEBUG2(dec, HEX);
        double dra = (ra * 24.) / 65536.;
        double ddec = (dec * 360.) / 65536.;
        DEBUG("=");
        DEBUG2(dra, 6);
        DEBUG(":");
        DEBUG2(ddec, 6);
        DEBUGLN();
        telescope.gotoRaDec(dra, ddec);
        return respond();
      }
      break;
    case 'r': // GOTO precise RA/DEC
      if (cnt == 18)
      {
        // DEBUG("GoTo to precise RA/DEC:");
        // “r34AB0500,12CE0500”
        unsigned long ra = (readHex(buffer[1], buffer[2]) << 16) + (readHex(buffer[3], buffer[4]) << 8) + readHex(buffer[5], buffer[6]);
        unsigned long dec = (readHex(buffer[10], buffer[11]) << 16) + (readHex(buffer[12], buffer[13]) << 8) + readHex(buffer[14], buffer[15]);
        // DEBUG2(ra, HEX);
        // DEBUG(":");
        // DEBUG2(dec, HEX);
        double dra = (ra * 24.) / (256 * 65536.);
        double ddec = (dec * 360.) / (256 * 65536.);
        // DEBUG("=");
        // DEBUG2(dra, 6);
        // DEBUG(":");
        // DEBUG2(ddec, 6);
        // DEBUGLN();
        telescope.gotoRaDec(dra, ddec);
        return respond();
      }
      break;
    case 'B': // GOTO AZM-ALT
      if (cnt == 10)
      {
        DEBUG("GoTo to AZM-ALT:");
        // “B12AB,4000”
        unsigned long alt = (readHex(buffer[1], buffer[2]) << 8) + readHex(buffer[3], buffer[4]);
        unsigned long azi = (readHex(buffer[6], buffer[7]) << 8) + readHex(buffer[8], buffer[9]);
        DEBUG2(alt, HEX);
        DEBUG(":");
        DEBUG2(azi, HEX);
        DEBUGLN();
        telescope.gotoAltAzi((alt * 360.) / 65536., (azi * 360.) / 65536.);
        return respond();
      }
      break;
    case 'b': // GOTO precise AZM-ALT
      if (cnt == 18)
      {
        DEBUG("GoTo to precise AZM-ALT:");
        // “b12AB0500,40000500”
        unsigned long alt = (readHex(buffer[1], buffer[2]) << 16) + (readHex(buffer[3], buffer[4]) << 8) + readHex(buffer[5], buffer[6]);
        unsigned long azi = (readHex(buffer[10], buffer[11]) << 16) + (readHex(buffer[12], buffer[13]) << 8) + readHex(buffer[14], buffer[15]);
        DEBUG2(alt, HEX);
        DEBUG(":");
        DEBUG2(azi, HEX);
        double dalt = (alt * 360.) / (256 * 65536.);
        double dazi = (azi * 360.) / (256 * 65536.);
        DEBUG("=");
        DEBUG2(dalt, 6);
        DEBUG(":");
        DEBUG2(dazi, 6);
        DEBUGLN();
        telescope.gotoAltAzi(dalt, dazi);
        return respond();
      }
      break;
    }
    return false;
  }

  bool procSync()
  {
    switch (buffer[0])
    {
    case 'S': // Sync RA/DEC
      if (cnt == 10)
      {
        DEBUG("Sync to RA/DEC:");
        DEBUGLN();
        // “S34AB,12CE”
        long ra = (readHex(buffer[1], buffer[2]) << 16) + (readHex(buffer[3], buffer[4]) << 8) + readHex(buffer[5], buffer[6]);
        long dec = (readHex(buffer[10], buffer[11]) << 16) + (readHex(buffer[12], buffer[13]) << 8) + readHex(buffer[14], buffer[15]);
        DEBUG2(ra, HEX);
        DEBUG(":");
        DEBUG2(dec, HEX);
        double dra = (ra * 24.) / (256 * 65536.);
        double ddec = (dec * 360.) / (256 * 65536.);
        DEBUG("=");
        DEBUG2(dra, 6);
        DEBUG(":");
        DEBUG2(ddec, 6);
        DEBUGLN();
        telescope.syncCurrentPosToRaDec(dra, ddec);
        return respond();
      }
      break;
    case 's': // Sync to precise RA/DEC
      if (cnt == 18)
      {
        DEBUG("Sync to precise RA/DEC:");
        DEBUGLN();
        // “s34AB0500,12CE0500”
        long ra = (readHex(buffer[1], buffer[2]) << 16) + (readHex(buffer[3], buffer[4]) << 8) + readHex(buffer[5], buffer[6]);
        long dec = (readHex(buffer[10], buffer[11]) << 16) + (readHex(buffer[12], buffer[13]) << 8) + readHex(buffer[14], buffer[15]);
        DEBUG2(ra, HEX);
        DEBUG(":");
        DEBUG2(dec, HEX);
        DEBUGLN();
        double dra = (ra * 24.) / (256 * 65536.);
        double ddec = (dec * 360.) / (256 * 65536.);
        telescope.syncCurrentPosToRaDec(dra, ddec);
        return respond();
      }
      break;
    }
    return false;
  }

  /*
      0 = Off
      1 = Alt/Az
      2 = EQ North
      3 = EQ South
    */
  bool procTracking()
  {
    switch (buffer[0])
    {
    case 't': // Get Tracking Mode
      // chr(mode) & “#”
      return respond(telescope.getTrackingMode());
    case 'T': // Set Tracking Mode
      if (cnt == 2)
      {
        // “T” & chr(mode) “
        return respond();
      }
      break;
    }
    return false;
  }

  bool procSlewing()
  {
    if (buffer[0] == 'P' && cnt == 8)
    {
      DEBUGLN("Slewing");

      /*
        For variable rates, multiply the desired rate by 4 and then separate it into a high and low byte. For example if the
        desired tracking rate is 150 arcseconds/second, then:
        trackRateHigh = (150 * 4) \ 256 = 2, and
        trackRateLow = (150 * 4) mod 256 = 88
      */

      if (buffer[1] == 3 && buffer[2] == 16 && buffer[3] == 6)
      { // Variable rate Azm (or RA) slew in positive direction
        int trackRateHigh = buffer[4];
        int trackRateLow = buffer[5];
        // buffer[6] && buffer[7] == 0
        telescope.aziAuto = (1. / (60 * 60)) * (trackRateLow + trackRateHigh * 256) / 4;
        DEBUG3LN("Variable rate Azm (or RA) slew in positive direction", trackRateHigh * 256 + trackRateLow, DEC);
        return respond();
      }
      if (buffer[1] == 3 && buffer[2] == 16 && buffer[3] == 7)
      { // Variable rate Azm (or RA) slew in negative direction
        int trackRateHigh = buffer[4];
        int trackRateLow = buffer[5];
        // buffer[6] && buffer[7] == 0
        telescope.aziAuto = -(1. / (60 * 60)) * (trackRateLow + trackRateHigh * 256) / 4;
        DEBUG3LN("Variable rate Azm (or RA) slew in negative direction", trackRateHigh * 256 + trackRateLow, DEC);
        return respond();
      }
      if (buffer[1] == 3 && buffer[2] == 17 && buffer[3] == 6)
      { // Variable rate Alt (or Dec) slew in positive direction
        int trackRateHigh = buffer[4];
        int trackRateLow = buffer[5];
        // buffer[6] && buffer[7] == 0
        telescope.altAuto = (1. / (60 * 60)) * (trackRateLow + trackRateHigh * 256) / 4;
        DEBUG3LN("Variable rate Alt (or Dec) slew in positive direction", trackRateHigh * 256 + trackRateLow, DEC);
        return respond();
      }
      if (buffer[1] == 3 && buffer[2] == 17 && buffer[3] == 7)
      { // Variable rate Alt (or Dec) slew in negative direction
        int trackRateHigh = buffer[4];
        int trackRateLow = buffer[5];
        // buffer[6] && buffer[7] == 0
        telescope.altAuto = -(1. / (60 * 60)) * (trackRateLow + trackRateHigh * 256) / 4;
        DEBUG3LN("Variable rate Alt (or Dec) slew in negative direction", trackRateHigh * 256 + trackRateLow, DEC);
        return respond();
      }

      /*
      For fixed rates, simply use a value from 1-9 (or 0 to stop) to mimic the equivalent hand control rates.
      Note: in most configurations, issuing the slew commands will override (or conflict with) the tracking mode of the
      mount. Hence it is always best to disable tracking first using the Tracking Commands, issue the slew command, then
      re-enable tracking. The main exception to this is when tracking equatorially - the fixed rate slews at 1 or 2 will not
      override tracking. This can be useful to simulate autoguiding.
      Note: on GT models, the fixed rate slews at rate 9 move at 3 degrees per second instead of the maximum rate.
       */
      if (buffer[1] == 2 && buffer[2] == 16 && buffer[3] == 36)
      { // Fixed rate Azm (or RA) slew in positive direction
        int rate = buffer[4];
        // buffer[5] && buffer[6] && buffer[7] == 0
        telescope.aziSlew = rate;
        DEBUG3LN("Fixed rate Azm (or RA) slew in positive direction", rate, DEC);
        return respond();
      }
      if (buffer[1] == 2 && buffer[2] == 16 && buffer[3] == 37)
      { // Fixed rate Azm (or RA) slew in negative direction
        int rate = buffer[4];
        // buffer[5] && buffer[6] && buffer[7] == 0
        telescope.aziSlew = -rate;
        DEBUG3LN("Fixed rate Azm (or RA) slew in negative direction", rate, DEC);
        return respond();
      }
      if (buffer[1] == 2 && buffer[2] == 17 && buffer[3] == 36)
      { // Fixed rate Alt (or Dec) slew in positive direction
        int rate = buffer[4];
        // buffer[5] && buffer[6] && buffer[7] == 0
        telescope.altSlew = rate;
        DEBUG3LN("Fixed rate Alt (or Dec) slew in positive direction", rate, DEC);
        return respond();
      }
      if (buffer[1] == 2 && buffer[2] == 17 && buffer[3] == 37)
      { // Fixed rate Alt (or Dec) slew in negative direction
        int rate = buffer[4];
        // buffer[5] && buffer[6] && buffer[7] == 0
        telescope.altSlew = -rate;
        DEBUG3LN("Fixed rate Alt (or Dec) slew in negative direction", rate, DEC);
        return respond();
      }
    }
    return false;
  }

  /*
      The format of the time commands is: QRSTUVWX, where:
      Q is the hour (24 hour clock).
      R is the minutes.
      S is the seconds.
      T is the month.
      U is the day.
      V is the year (century assumed as 20).
      W is the offset from GMT for the time zone. Note: if zone is negative, use 256-zone.
      X is 1 to enable Daylight Savings and 0 for Standard Time.
    */
  bool procTime()
  {
    tmElements_t tm;

    switch (buffer[0])
    {
    case 'h': // Get Time
      if (cnt == 1)
      {
        time_t t = telescope.astro.getCurrentTime();
        DEBUG("Current time is ");
        DEBUG2(t, DEC);
        DEBUG(" relative: ");
        DEBUG2(t - millis(), DEC);
        DEBUGLN();
        if (useDST)
        {
          t += 1 * 60 * 60; // plus one hour
        }
        t += GMToffset * 60 * 60; // plus GMToffset hours
        // chr(Q) &chr(R) &chr(S) &chr(T) &chr(U) &chr(V) &chr(W) &chr(X) &“#”
        return respond(hour(t), minute(t), second(t), month(t), day(t), year(t) - 2000, 256 + GMToffset, useDST ? 1 : 0);
        // return respond();
      }
      break;
    case 'H': // Set Time
      // DEBUG("SetTime received0,"); DEBUG2(cnt,DEC); DEBUGLN();
      if (cnt == 9)
      {
        // “H” &chr(Q) &chr(R) &chr(S) &chr(T) &chr(U) &chr(V) &chr(W) &chr(X)
        tm.Hour = buffer[1];
        tm.Minute = buffer[2];
        tm.Second = buffer[3];
        tm.Month = buffer[4];
        tm.Day = buffer[5];
        tm.Year = buffer[6] + 2000 - 1970;
        GMToffset = buffer[7];
        if (GMToffset >= 128)
        {
          GMToffset -= 256;
        }
        useDST = buffer[8];
        time_t t = makeTime(tm);
        if (useDST)
        {
          t -= 1 * 60 * 60;
        }
        t -= GMToffset * 60 * 60; // plus GMToffset hours;

        DEBUG3LN("SetTime received:", t, DEC);

        telescope.astro.setCurrentTime(t); // in seconds

        // debug
        {
          time_t tt = (time_t)(telescope.astro.getCurrentTime());
          DEBUG2(telescope.astro.getCurrentTime(), DEC);
          DEBUG4(" y m d h m s ", year(tt), month(tt), DEC);
          DEBUG4(",", day(tt), hour(tt), DEC);
          DEBUG4(",", minute(tt), second(tt), DEC);
          DEBUG3(" sid ", telescope.astro.getLocalSiderealTime(), 6);
        }

        return respond();
      }
      break;
    }
    return false;
  }

  /*
      The format of the location commands is: ABCDEFGH, where:
      A is the number of degrees of latitude.
      B is the number of minutes of latitude.
      C is the number of seconds of latitude.
      D is 0 for north and 1 for south.
      E is the number of degrees of longitude.
      F is the number of minutes of longitude.
      G is the number of seconds of longitude.
      H is 0 for east and 1 for west.
    */
  bool procLocation()
  {
    switch (buffer[0])
    {
    case 'w': // Get Location
      if (cnt == 1)
      {
        // chr(A) &chr(B) &chr(C) &chr(D) &chr(E) &chr(F) &chr(G) &chr(H) &“#”
        double wgsLat = telescope.astro.getLatDec();  // is in degrees
        double wgsLon = telescope.astro.getLongDec(); // is in degrees
        // DEBUG4LN("GetLocation=",wgsLat,wgsLon, DEC);
        return respond(
            ((uint8_t)(abs(wgsLat))), ((uint8_t)((int)(wgsLat * 60) % 60)), ((uint8_t)((int)(wgsLat * 60 * 60) % 60)), wgsLat < 0 ? 1 : 0,
            ((uint8_t)(abs(wgsLon))), ((uint8_t)((int)(wgsLon * 60) % 60)), ((uint8_t)((int)(wgsLon * 60 * 60) % 60)), wgsLon < 0 ? 1 : 0);
      }
      break;
    case 'W': // Set Location
      if (cnt == 9)
      {
        // “W” &chr(A) &chr(B) &chr(C) &chr(D) &chr(E) &chr(F) &chr(G) &chr(H)
        // DEBUG("SetLocation");
        // DEBUGLN();
        double wgsLat = buffer[1] + (buffer[2] / 60.) + (buffer[3] / (60 * 60.));
        if (buffer[4] == 1)
        {
          wgsLat *= -1;
        }
        double wgsLon = buffer[5] + (buffer[6] / 60.) + (buffer[7] / (60 * 60.));
        if (buffer[8] == 1)
        {
          wgsLat *= -1;
        }
        // DEBUG4LN("SetLocation:",wgsLat,wgsLon,6);
        telescope.astro.setLatLong(wgsLat, wgsLon);
        return respond();
      }
      break;
    }
    return false;
  }

  bool procGps()
  {
    if (buffer[0] == 'P' && cnt == 8 && buffer[1] == 1 && buffer[2] == 176)
    {
      DEBUG("GPS command");
      DEBUGLN();
      if (buffer[3] == 55 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 1)
      {
        // Is GPS Linked? X > 0 if linked, 0 if not linked
        // chr(x) &“#”
        return respond(telescope.isGpsLinked() ? 1 : 0);
      }
      if (buffer[3] == 1 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 3)
      {
        // Get Latitude ((x*65536)+(y*256)+z)/(2^24) is a fraction of a rotation. To convert to degrees, multiply by 360.
        // chr(x) & chr(y) & chr(z) &“#”
        double wgsLat = telescope.astro.getLatDec(); // is in degrees
        uint32_t xyz = ((1L << 24) / 360) * wgsLat;
        uint8_t x = xyz >> 16;
        uint8_t y = xyz >> 8;
        uint8_t z = xyz;
        return respond(x, y, z);
      }
      if (buffer[3] == 2 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 3)
      {
        // Get Longitude ((x*65536)+(y*256)+z)/(2^24) is a fraction of a rotation. To convert to degrees, multiply by 360.
        // chr(x) & chr(y) & chr(z) &“#”
        double wgsLon = telescope.astro.getLongDec(); // is in degrees
        uint32_t xyz = ((1L << 24) / 360) * wgsLon;
        uint8_t x = xyz >> 16;
        uint8_t y = xyz >> 8;
        uint8_t z = xyz;
        return respond(x, y, z);
      }
      if (buffer[3] == 3 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 2)
      {
        // Get Date x is month (1-12) y is day (1-31)
        // chr(x) & chr(y) & “#”
        time_t t = telescope.astro.getCurrentTime();
        return respond(month(t), day(t));
      }
      if (buffer[3] == 4 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 2)
      {
        // Get Year (x * 256) + y = year
        // chr(x) & chr(y) & “#”
        time_t t = telescope.astro.getCurrentTime();
        return respond(year(t) / 256, year(t) % 256);
      }
      if (buffer[3] == 51 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 3)
      {
        // Get Time  x is the hours y is the minutes z is the seconds
        // chr(x) & chr(y) & chr(z) &“#”
        time_t t = telescope.astro.getCurrentTime();
        return respond(hour(t), minute(t), second(t));
      }
    }
    return false;
  }

  bool procRTC()
  {
    if (buffer[0] == 'P' && cnt == 8 && buffer[1] == 1 && buffer[2] == 178)
    {
      DEBUG("RTC command");
      DEBUGLN();
      if (buffer[3] == 3 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 2)
      {
        // Get Date x is month (1-12) y is day (1-31)
        // chr(x) &chr(y) & “#”
        return respond();
      }
      if (buffer[3] == 4 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 2)
      {
        // Get Year (x * 256) + y = year
        // chr(x) & chr(y) &“#”
        return respond();
      }
      if (buffer[3] == 51 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 3)
      {
        // Get Time x is the hours y is the minutes z is the seconds
        // chr(x) & chr(y) & chr(z) &“#”
        return respond();
      }
    }
    if (buffer[0] == 'P' && cnt == 8 && buffer[1] == 3 && buffer[2] == 178)
    {
      if (buffer[3] == 131 && buffer[6] == 0 && buffer[7] == 0)
      {
        // Set Date x is month (1-12) y is day (1-31)
        // int x = buffer[4];
        // int y = buffer[5];
        return respond();
      }
      if (buffer[3] == 132 && buffer[6] == 0 && buffer[7] == 0)
      {
        // Set Year (x * 256) + y = year
        // int x = buffer[4];
        // int y = buffer[5];
        return respond();
      }
    }
    if (buffer[0] == 'P' && cnt == 8 && buffer[1] == 4 && buffer[2] == 178)
    {
      if (buffer[3] == 179 && buffer[7] == 0)
      {
        // Set Time x is the hours y is the minutes z is the seconds
        // int x = buffer[4];
        // int y = buffer[5];
        // int z = buffer[6];
        return respond();
      }
    }
    return false;
  }

  bool procMisc()
  {
    switch (buffer[0])
    {
    case 'V': // Get Version
      if (cnt == 1)
      {
        DEBUG("GetVersion?");
        DEBUGLN();
        int major = 1;
        int minor = 1;
        // chr(major) & chr(minor) & “#”
        return respond(major, minor);
      }
      break;
    case 'P': // Get Device Version Devices include:  16 = AZM/RA Motor 17 = ALT/DEC Motor 176 = GPS Unit 178 = RTC (CGE only)
      //        “P” &           //chr(1) &          //chr(dev) &          //chr(254) &          //chr(0) &          //chr(0) &          //chr(0) &          //chr(2)
      if (cnt == 8 && buffer[1] == 1 && buffer[3] == 254 && buffer[4] == 0 && buffer[5] == 0 && buffer[6] == 0 && buffer[7] == 2)
      {
        DEBUG3LN("GetDeviceVersion?", buffer[2], DEC);
        // chr(major) & chr(minor) & “#”
        int major = 1;
        int minor = 1;
        uint8_t dev = buffer[2];
        switch (dev)
        {
        case 16: //  AZM/RA Motor
          return respond(major, minor);
        case 17: //  ALT/DEC Motor
          return respond(major, minor);
        case 176: //  GPS Unit
          return respond(major, minor);
        case 178: //  RTC (CGE only)
          return respond(major, minor);
        }
      }
      break;
    case 'm': // Get Model
      // 1 = GPS Series
      // 3 = i-Series
      // 4 = i-Series SE
      // 5 = CGE
      // 6 = Advanced GT =      C6-S C8-S C9.25-S  C11-S
      // 7 = SLT
      // 9 = CPC
      // 10 = GT
      // 11 = 4/5 SE
      // 12 = 6/8 SE
      //  chr(model) & “#”
      {
        DEBUGLN("GetModel?");
        int model = 6;
        return respond(model);
      }

    case 'K':
      DEBUG("Echo");
      DEBUGLN();
      if (cnt == 2)
      { // Echo - useful to check communication
        // “K” &chr(x)
        // chr(x) & “#”
        return respond(buffer[1]);
      }
      break;
    case 'J': // Is Alignment Complete? - align=1 if aligned and 0 if not
      // chr(align) &#
      DEBUGLN("IsAligned?");
      return respond(telescope.isAligned() ? 1 : 0);

    case 'L': // Is GOTO in Progress? - Response is ASCII “0” or “1”
      // prog & “#”
      // DEBUGLN("IsGoto?");
      return respond(((uint8_t)(telescope.isGotoInProgress() ? '1' : '0')));

    case 'M': // Cancel GOTO
      DEBUG("CancelGoto");
      DEBUGLN();
      telescope.cancelGoto();
      return respond();
    }
    return false;
  }

  bool procError()
  {
    int data = buffer[cnt - 1];
    return data == '\r' || data == '\n';
  }

public:
  NexstarProtocol(NexstarControl &tel, Stream &comm) : telescope(tel), comm(comm)
  {
    reset();
  }

  bool process(uint8_t data)
  {
    buffer[cnt++] = data;
    bool b = procGetPosition() || procGoto() || procSync() || procTracking() || procSlewing() || procTime() || procLocation() || procGps() || procRTC() || procMisc(); // || procError();

    if (b)
    {
      // DEBUG("True returned");
      // DEBUGLN();
      reset();
    }
    return b;
  }

  void reset()
  {
    cnt = 0;
    // DEBUG("Resetting CNT");
    // DEBUGLN();
  }

  bool isPending()
  {
    return cnt != 0;
  }
};
