#pragma once

#include <Arduino.h>

#include "astro.h"
#include "chassis.h"
#include "nexstar.h"
#include "debug.h"

class TelescopeControl : public NexstarControl
{

private:
    /**
     * deg, -90 to 90, not minutes but fraction of degrees
     */
    // double altitude = 45;
    /**
     * deg, -180 to 180, not minutes but fraction of degrees from north
     */
    // double azimuth = 16;

    // GPS position, location on Earth
    // double wgsLon = 19.1;
    // double wgsLat = 47.5;
public:
    unsigned long timeBase; // at the moment of millis() == 0

    bool _tracking = false;
    double trackRA = 0;
    double trackDEC = 0;
    bool aligned = false;

    double targetAzimuth = 0;
    double targetAltitude = 0;

    bool gotoInProgress = false;
public:
    MyAstro &myAstro;
    MyChassis &myChassis;

public:
    TelescopeControl(MyAstro &_myAstro, MyChassis &_myChassis) : myAstro(_myAstro), myChassis(_myChassis)
    {
        // bool b = begin();
        // DEBUG("Astro inited:");
        // DEBUG(b?"OK":"Fail");
        // DEBUGLN();

        // rejectDST();
        // setAltAz(45.0, 19.);
        // doAltAz2RAdec();
        // if( b ) {
        //     DEBUGLN("Astro TRUE");
        // }
    }

public:
    /**
     * @brief updates astro position from chassis data
     * 
     * Takes chassis setting and applies to astro position
     */
    void updateChassis()
    {
        myAstro.applyAltAz(myChassis.getAltCurrent(), myChassis.getAziCurrent());
        // doAltAz2RAdec();
    }

    void cancelGoto()
    {
    }

    bool isGpsLinked()
    {
        // TODO check GPS
        return false;
    }

    int getTrackingMode()
    {
        // 0 = Off
        // 1 = Alt/Az
        // 2 = EQ North
        // 3 = EQ South
        DEBUGLN("getTrackingMode");
        if (_tracking)
            return 1;
        return 0;
    }

    inline bool isGotoInProgress() { return myChassis.isGoToInProgress(); }
    inline bool isAligned() { return aligned; }

    void gotoAltAzi(double alt, double azi)
    {
        // altitude = alt;
        // azimuth = azi;
        myAstro.applyAltAz(alt, azi);
        // doAltAz2RAdec();
        DEBUG4LN("GO TO ALT/AZI:", alt, azi, 4);
        myChassis.gotoAltAzi(myAstro.getAltitude(), myAstro.getAzimuth());
    }

    void gotoRaDec(double ra, double dec)
    {
        myAstro.applyRAdec(ra, dec);
        trackDEC = dec;
        trackRA = ra;
        // doRAdec2AltAz();
        DEBUG4("GO TO RA/DEC:", ra, dec, 4);
        DEBUG4LN(" Alt=", myAstro.getAltitude(), myAstro.getAzimuth(), 4);
        myChassis.gotoAltAzi(myAstro.getAltitude(), myAstro.getAzimuth());
    }

    void syncCurrentPosToRaDec(double ra, double dec)
    {
        DEBUG3("Sync:RA=", ra, 4);
        DEBUG3(",DEC=", dec, 4);
        myAstro.applyRAdec(ra, dec);
        // doRAdec2AltAz();
        DEBUG3(",RA=:", myAstro.getRAdec(), 4);
        DEBUG3(",Dec=", myAstro.getDeclinationDec(), 4);
        DEBUG3(",Alt=:", myAstro.getAltitude(), 4);
        DEBUG3LN(",Azi=", myAstro.getAzimuth(), 4);
        myChassis.syncAltAzi(myAstro.getAltitude(), myAstro.getAzimuth());
        trackDEC = dec;
        trackRA = ra;
        aligned = true;
    }

private:
    int counter = 0;

public:
    void stopSlew()
    {
        altAuto = 0;
        aziAuto = 0;
        altSlew = 0;
        aziSlew = 0;
        DEBUGLN("Stopping slew.");
        setTracking(false);
    }

    void tick()
    {
        if (counter++ == 0x10)
        {
            // very slow tick
            // time_t tm = getCurrentTime();
            if (isTracking())
            {
                // setRAdec(trackRA, trackDEC);  // recalculate actual ALT/AZI from pre-set RA and DEC
                gotoRaDec(trackRA, trackDEC); // recalculate actual ALT/AZI from pre-set RA and DEC
            }

            counter = 0;
            if (altAuto != 0 || altSlew != 0 || aziAuto != 0 || aziSlew != 0)
            {
                const double autoRate = 0.1;
                const double slewRate = 0.1;
                gotoAltAzi(myAstro.getAltitude() + autoRate * altAuto + slewRate * altSlew, myAstro.getAzimuth() + autoRate * aziAuto + slewRate * aziSlew);
            }
        }
    }

    bool isTracking()
    {
        return _tracking;
    }

    void setTracking(bool bOn)
    {
        if (!_tracking && bOn)
        {
            // save RA and DEC
            trackRA = myAstro.getRAdec();
            trackDEC = myAstro.getDeclinationDec();
        }
        _tracking = bOn;
        DEBUG("Tracking:");
        DEBUGLN(bOn ? "ON" : "OFF");
    }
};
