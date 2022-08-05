#pragma once

#include <Arduino.h>

#include "debug.h"
#include "astro.h"
#include "chassis.h"
#include "nexstar.h"
#include "loadsave.h"

/**
 * coordinated control of telescope
 *
 *
 */
class TelescopeControl : public NexstarControl
{
    // public:
    unsigned long timeBase; // at the moment of millis() == 0

    bool _tracking = false;
    double trackRA = 0;
    double trackDEC = 0;
    bool aligned = false;

    double targetAzimuth = 0;
    double targetAltitude = 0;

    bool gotoInProgress = false;

public:
    MyAstro astro;
    MyChassis &chassis;

public:
    TelescopeControl(MyChassis &_myChassis) : NexstarControl(astro), chassis(_myChassis)
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

#ifdef _DEBUG
        // saveLatLonToEEPROM(47.5, 19.1);
#endif
        // read stored latitude and longitude
        astro.setLatLong(readLatFromEEPROM(), readLonFromEEPROM());
    }

public:
    /**
     * @brief updates astro position from chassis data
     *
     * Takes chassis setting and applies to astro position
     */
    void updateChassis()
    {
        astro.applyAltAz(chassis.getAltCurrent(), chassis.getAziCurrent());
        // doAltAz2RAdec();
    }

    void cancelGoto()
    {
        chassis.stop();
        stopSlew();
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
        DEBUG4LN((isTracking() ? "getTrackingMode:ON" : "getTrackingMode:OFF"), trackRA, trackDEC, DEC);
        if (isTracking())
        {
            return 1;
        }
        return 0;
    }

    inline bool isGotoInProgress() { return chassis.isGoToInProgress(); }
    inline bool isAligned() { return aligned; }

    void gotoAltAzi(double alt, double azi)
    {
        // altitude = alt;
        // azimuth = azi;
        astro.applyAltAz(alt, azi);
        // doAltAz2RAdec();
        DEBUG4LN("GO TO ALT/AZI:", alt, azi, 4);
        chassis.gotoAltAzi(astro.getAltitude(), astro.getAzimuth());
    }

    void gotoRaDec(double ra, double dec)
    {
        if (!astro.applyRAdec(ra, dec))
        {
            DEBUGLN("RADec not updated");
        }
        else
        {
            trackDEC = dec;
            trackRA = ra;
            // doRAdec2AltAz();
            DEBUG4("GO TO RA/DEC:", ra, dec, 4);
            DEBUG4LN(" Alt=", astro.getAltitude(), astro.getAzimuth(), 4);
            chassis.gotoAltAzi(astro.getAltitude(), astro.getAzimuth());
        }
    }

    void syncCurrentPosToRaDec(double ra, double dec)
    {
        DEBUG3("Sync:RA=", ra, 4);
        DEBUG3(",DEC=", dec, 4);
        astro.applyRAdec(ra, dec);
        // doRAdec2AltAz();
        DEBUG3(",RA=:", astro.getRAdec(), 4);
        DEBUG3(",Dec=", astro.getDeclinationDec(), 4);
        DEBUG3(",Alt=:", astro.getAltitude(), 4);
        DEBUG3LN(",Azi=", astro.getAzimuth(), 4);
        chassis.syncAltAzi(astro.getAltitude(), astro.getAzimuth());
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
                gotoAltAzi(astro.getAltitude() + autoRate * altAuto + slewRate * altSlew, astro.getAzimuth() + autoRate * aziAuto + slewRate * aziSlew);
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
            trackRA = astro.getRAdec();
            trackDEC = astro.getDeclinationDec();
        }
        _tracking = bOn;
        DEBUG("Tracking:");
        DEBUGLN(bOn ? "ON" : "OFF");
    }
};
