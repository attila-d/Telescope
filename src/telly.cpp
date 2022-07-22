#include <AccelStepper.h>
#include <Arduino.h>
#include <OneButton.h>
#include <RotaryEncoder.h>

#include "debug.h"
#include "util.h"

#include "chassis.h"
#include "compass.h"
#include "handwheel.h"
#include "nexstar.h"
#include "loadsave.h"
#include "telescope.h"
#include "tilt.h"
// #include "gps.h"
#include "bt.h"
#include "telly.h"

void TellyMain::stopMotion()
{
    chassis.stop();
    telescope.stopSlew();
    actualState = Normal;
}

void TellyMain::startSpiral()
{
    actualState = Spiral00;
    spiralFactor = 1.;
    chassis.stepAlt.move(0.5 * stepSize * spiralFactor / 8);
}

void TellyMain::setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
#ifdef _DEBUG
    DEBUG("Starting...");
    // DEBUG("Loop size in steps:");
    // DEBUG2(STEPS_PER_REVOLUTION, DEC);
    DEBUGLN();
#endif //_DEBUG
#ifdef __BT
    // Serial3.begin(9600);
    Serial3.begin(115200);
    initBT(protocol, &Serial3);
#endif
#ifdef __GPS
    initGPS();
#endif // __GPS

#ifdef __ACCEL
    initADXL345();
#endif // __ACCEL

#ifdef __MAGNETO
    initMPU9250();
#endif

    initializeTelescope();
}

void TellyMain::slowTick()
{
    handwheel.tick();

#ifdef __GPS
    if (!gpsFix())
    {
        tickGPS();
        if (isGpsFix())
        {
            gpsFix = true; // we don't use it anymore
            float lat = getGpsLat();
            float lon = getGpsLon();
            time_t tm = getGpsDateTime();
            DEBUG3("GPS Fixed,", lat, 6);
            DEBUG3(":", lon, 6);
            DEBUG3(", time=", tm, DEC);
            DEBUGLN("!");
            setLatLong(lat, lon);
            telescope.setCurrentTime(tm);
            saveLatLonToEEPROM(lat, lon);
        }
    }
#endif // __GPS

#ifdef __BT
    // bluetooth
    tickBT();
#endif // __BT

#ifdef __ACCEL
    // DEBUG("Tilt:");
    // DEBUG2LN(TILT(),2);
    {
        float tilt = TILT();
        if (tilt < 0)
        {
            chassis.stopAlt();
            DEBUGLN("Stopping ALT motor");
        }
    }
#endif

#ifdef _DEBUG
    if (Serial.available() > 0)
    {
        int ch = Serial.read();
        switch (ch)
        {
        case 'a':
            // print step position
            DEBUG3("Chassis AltAzi Step Pos=", chassis.stepAlt.currentPosition(), DEC);
            DEBUG3(":", chassis.stepAzi.currentPosition(), DEC);
            DEBUG3("\tOffset=", chassis.altStepOffset, DEC);
            DEBUG3(":", chassis.aziStepOffset, DEC);
            DEBUG3("\tRaw Pos=", chassis.stepAlt.currentPosition(), DEC);
            DEBUG3(":", chassis.stepAzi.currentPosition(), DEC);
            DEBUG3("\tTrg=", chassis.stepAlt.targetPosition(), DEC);
            DEBUG3(":", chassis.stepAzi.targetPosition(), DEC);
            DEBUG(attachHandwheel ? "\tAttached" : "\tDetached");
            DEBUG3("\tNormalized Pos=", chassis.getAltCurrent(), 4);
            DEBUG3(":", chassis.getAziCurrent(), 4);
            DEBUG3("\tTarget=", chassis.getAltTarget(), 4);
            DEBUG3LN(":", chassis.getAziTarget(), 4);
            break;
        case 'l':
            // print GPS location
            DEBUG("Location WgsLatLon=");
            DEBUG2(astro.getLatDec(), 6);
            DEBUG(":");
            DEBUG2(astro.getLongDec(), 6);
            DEBUGLN();
            break;
        case 'p':
            // print target position
            DEBUG("Pointing to Alt/Azi/Ra/Dec=");
            DEBUG2(astro.getAltitude(), 6);
            DEBUG(":");
            DEBUG2(astro.getAzimuth(), 6);
            DEBUG(", ");
            DEBUG2(astro.getRAdec(), 6);
            DEBUG(":");
            DEBUG2(astro.getDeclinationDec(), 6);
            DEBUGLN();
            break;
        case 's':
            // print sensor data
            // DEBUG("Sensor data:");
#ifdef __GPS
            DEBUG(isGpsFix() ? "GPS FIX OK, " : "NOT FIXED!");
            DEBUG3(" Sat:", getGpsSatellites(), DEC);
            if (gpsFix)
            {
                DEBUG3(",Lat=", getGpsLat(), 6);
                DEBUG3(",Lon=", getGpsLon(), 6);
                DEBUG3(",Date:", getGpsDateTime(), DEC);
            }
#endif // __GPS
#ifdef __MAGNETO
            DEBUG(" North:");
            DEBUG2LN(NORTH(), 2);
            DEBUG("Mag:"); // rotation around axes, Z means AZIMUTH rotation
            DEBUG3(",x=", getMPU9250().mx, 6);
            DEBUG3(",y=", getMPU9250().my, 6);
            DEBUG3LN(",z=", getMPU9250().mz, 6);
            // DEBUG("Gyro:");  // rotation around axes, Z means AZIMUTH rotation
            // DEBUG3(",x=", getMPU9250().gx, 6);
            // DEBUG3(",y=", getMPU9250().gy, 6);
            // DEBUG3LN(",z=", getMPU9250().gz, 6);
            DEBUG("Accel:");
            DEBUG3(",x=", getMPU9250().ax, 6);
            DEBUG3(",y=", getMPU9250().ay, 6);
            DEBUG3LN(",z=", getMPU9250().az, 6);
#endif // __MAGNETO
#ifdef __ACCEL
            DEBUG(" Tilt:");
            DEBUG2LN(TILT(), 2);
#endif // __ACCEL

            DEBUG4(" y m d h m s ", year(astro.getCurrentTime()), month(astro.getCurrentTime()), DEC);
            DEBUG4(",", day(astro.getCurrentTime()), hour(astro.getCurrentTime()), DEC);
            DEBUG4(",", minute(astro.getCurrentTime()), second(astro.getCurrentTime()), DEC);
            DEBUG3(" sid ", astro.getLocalSiderealTime(), 6);

            DEBUGLN();

            break;
        case 'n':
            // go to northpole (appx.)
            chassis.gotoAltAzi(45., 0.);
            break;
        default:
            if (ch <= '9' && ch >= '0')
            {
                int key = (ch - '0');
                // DEBUG("Go to degree: ");
                // // DEBUG2((key*90/10),DEC);
                // // DEBUG(" as step:");
                // // DEBUG2(calcAziSteps(key*36),DEC);
                // DEBUG2(((long)((key / 10.) * (altMax - altMin))), DEC);
                // // DEBUG(" range:");
                // // DEBUG2(((long)(aziMax-aziMin)),DEC);
                // DEBUGLN();
                // telescope.gotoAltAzi(key * 90 / 10, 0);
                // // gotoSteps((key/10.)*(altMax-altMin),0);

                // azimuth
                DEBUG3("Go to degree: ", (key * 36), DEC);
                DEBUG3(" as step:", ((long)((key / 10.) * (aziMax - aziMin))), DEC);
                DEBUG3LN("  range:", ((long)(aziMax - aziMin)), DEC);
                telescope.gotoAltAzi(90, key * 360 / 10);
                // gotoSteps(0, (key/10.)*(aziMax-aziMin));
            }
            else
            {
                DEBUG("a = chassis alt/azi step\ns = print sensor data\n");
                DEBUGLN("l = location wgs");
                DEBUGLN("p = telescope pointing");
                DEBUGLN("n = northpole");
                DEBUGLN("0..9 azi pos (360/x)");
                DEBUGLN();
            }
            break;
        }
    }
#endif

    switch (actualState)
    {
    case TellyState::Calibration:
    {
#ifdef __MAGNETO
        // DEBUG4LN("Mag tick:",stepAzi.currentPosition(), stepAzi.targetPosition(), DEC)
        tickMagCalibration();
        if (chassis.isAtTargetAzi())
        {
            // we got enough sampling data
            endMagCalibration();
            // TODO fix initial offsets
            actualState = TellyState::Normal;
        }
#endif
    }
    break;
    case Spiral00:
        if (chassis.isAtTargetAlt())
        {
            actualState = Spiral01;
            chassis.stepAzi.move(stepSize * spiralFactor / 8);
            // DEBUG3LN("Spiral01 Azi=",stepAzi.targetPosition(),DEC);
            spiralFactor += 0.25;
        }
        break;
    case Spiral01:
        if (chassis.isAtTargetAzi())
        {
            actualState = Spiral11;
            chassis.stepAlt.move(stepSize * spiralFactor / 8);
            // DEBUG3LN("Spiral11 Alt=",stepAlt.targetPosition(),DEC);
            spiralFactor += 0.25;
        }
        break;
    case Spiral11:
        if (chassis.isAtTargetAlt())
        {
            actualState = Spiral10;
            chassis.stepAzi.move(-stepSize * spiralFactor / 8);
            // DEBUG3LN("Spiral10 Azi=",stepAzi.targetPosition(),DEC);
            spiralFactor += 0.25;
        }
        break;
    case Spiral10:
        if (chassis.isAtTargetAzi())
        {
            actualState = Spiral00;
            chassis.stepAlt.move(-stepSize * spiralFactor / 8);
            // DEBUG3LN("Spiral00 Alt=",stepAlt.targetPosition(),DEC);
            spiralFactor += 0.25;
        }
        if (spiralFactor > 10)
        {
            stopMotion();
        }
        break;
    default:
#ifdef __MAGNETO
//            tickMPU9250();
#endif // __MAGNETO
        break;
    }

    telescope.tick();
}

void TellyMain::initializeTelescope()
{
#ifdef _DEBUG
    // saveLatLonToEEPROM(47.5, 19.1);
#endif
    // read stored latitude and longitude
    astro.setLatLong(readLatFromEEPROM(), readLonFromEEPROM());

    double alt = astro.getAltitude();
    double azi = astro.getAzimuth();
#ifdef __ACCEL
    DEBUG3("Initial tilt:", getTilt(), 4);
    alt = TILT();
    DEBUG3LN("\tReal tilt:", alt, 6);
#endif
#ifdef __MAGNETO
    tickMPU9250();
    // azi = lRange(360-NORTH(), 0, 360);
    azi = NORTH();
    DEBUG3LN("North=", azi, 4);
#endif // __MAGNETO
#ifdef __GPS
    // TODO: Wait for GPS fix?
    // Read Lat/Lon from previous run (EEPROM)
    // can not set time without GPS
#endif // __GPS

    astro.applyAltAz(alt, azi);
    chassis.syncAltAzi(alt, azi);
    DEBUG4("Alt/Azi:", astro.getAltitude(), astro.getAzimuth(), 4);
    DEBUG4LN("RA/Dec:", astro.getRAdec(), astro.getDeclinationDec(), 4);

    if (handwheel.initialAziButton == 0)
    {
        if (handwheel.initialAltButton == 0)
        {
            // both button pressed
            DEBUGLN("Calibrating magnetometer");
// calibrate magnetometer
#ifdef __MAGNETO
            actualState = Calibration;
#endif // __MAGNETO
        }
        else
        {
            DEBUGLN("To the top");
#ifdef __ACCEL
            actualState = MovingToTop;
#endif // __MAGNETO
        }
    }
    else
    {
        if (handwheel.initialAltButton == 0)
        {
            DEBUGLN("Move to north pole");
#ifdef __MAGNETO
#ifdef __ACCEL
            actualState = MovingToNorthPole;
#endif // __ACCEL
#endif // __MAGNETO
        }
        else
        {
            actualState = Normal;
        }
    }

    switch (actualState)
    {
    case TellyState::Normal:
        /* code */
        DEBUGLN("Starting normal operation");
        // stepAzi.move( (aziMax - aziMin) );   // test 360 rotation
        break;
    case TellyState::MovingToTop:
    {
        DEBUGLN("Going to top");
#ifdef __ACCEL
        DEBUG4LN("Top is:", alt, chassis.getAziCurrent(), 4);
        chassis.gotoAltAzi(90, chassis.getAziCurrent());
#endif
        actualState = Normal;
    }
    break;
    case TellyState::MovingToNorthPole:
    {
        // calculate alt and azi
        DEBUGLN("Going to north");
#ifdef __MAGNETO
        double alt = 90 - astro.getLatDec();
        DEBUG4LN("North is:", alt, 0., 4);
        chassis.gotoAltAzi(alt, 0);
#endif
        actualState = Normal;
    }
    break;
    case TellyState::Calibration:
    {
#ifdef __MAGNETO
        // stepAlt.moveTo(altMax-altStepOffset);
        chassis.stepAzi.move((aziMax - aziMin) * 1.1);
        beginMagCalibration();
#endif
    }
    break;

    default:
        break;
    }
}
