#include "main.h"

#include <Arduino.h>
#include <fp64lib.h>

#include <AccelStepper.h>
#include <OneButton.h>
#include <RotaryEncoder.h>

#include "chassis.h"
#include "compass.h"
#include "handwheel.h"
#include "nexstar.h"
#include "telescope.h"
#include "tilt.h"
#include "util.h"
// #include "gps.h"
#include "bt.h"
#include "telly.h"
#include "debug.h"

#define LED_BUILTIN 13

void debugfn();

// GPS: Rx2/Tx2
// BT: Rx3/Tx3 -> removed
// I2C: SCL1/SDA1
// I2C: 0x53 ADXL345 Triple-Axis Accelerometer,
// 0x68 MPU-9250 magneto 9DOF

RotaryEncoder encAlt(45, 47, RotaryEncoder::LatchMode::FOUR3);
RotaryEncoder encAzi(46, 48, RotaryEncoder::LatchMode::FOUR3);

AccelStepper stepAlt(AccelStepper::DRIVER, 25, 23); // driver, stepPin, dirPin;
AccelStepper stepAzi(AccelStepper::DRIVER, 24, 22); // driver, stepPin, dirPin;

MyChassis chassis(stepAlt, stepAzi);

OneButton btnAlt(ALT_BTN_PIN, true, true);
OneButton btnAzi(AZI_BTN_PIN, true, true);

Handwheel handwheel(encAlt, encAzi, btnAlt, btnAzi);

TellyMain telly(chassis, handwheel);

void setup()
{
    // pinMode(22, OUTPUT);
    // pinMode(23, OUTPUT);
    // pinMode(24, OUTPUT);
    // pinMode(25, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);

    chassis.setup();
    handwheel.setup();
    telly.setup();

    btnAlt.attachClick([]()
                       {
            switch (telly.actualState) {
                case Normal:
                    // if (handwheel.isAziPressed())  // AZI, the other button
                    if (handwheel.btnAzi.isLongPressed())  // AZI, the other button
                    {
                        telly.startSpiral();
                    } else {
                        telly.stopMotion();
                    }
                    break;
                case Spiral00:
                case Spiral01:
                case Spiral10:
                case Spiral11:
                    telly.stopMotion();
                    break;
                case Calibration:
                case MovingToNorthPole:
                case MovingToTop:
                    break;
                case Uninitialized:
                    break;
            } });
    btnAzi.attachClick([]()
                       {
            switch (telly.actualState) {
                case Normal:
                    // if (handwheel.isAltPressed())  // ALT, the other button
                    if (handwheel.btnAlt.isLongPressed())  // ALT, the other button
                    {
                        telly.startSpiral();
                    } else {
                        telly.stopMotion();
                    }
                    break;
                case Spiral00:
                case Spiral01:
                case Spiral10:
                case Spiral11:
                    telly.stopMotion();
                    break;
                case Calibration:
                case MovingToNorthPole:
                case MovingToTop:
                    break;
                case Uninitialized:
                    break;            
        } });
    btnAlt.attachDoubleClick([]()
                             {
        // detachTelescope();
        DEBUGLN("Tracking toggle");
        telly.telescope.setTracking(!telly.telescope.isTracking()); });

    btnAzi.attachDoubleClick([]()
                             {
        // attachTelescope();
        attachHandwheel = !attachHandwheel;
        DEBUG("Attaching telescope");
        DEBUGLN(attachHandwheel ? "ON" : "OFF"); });

    // debugfn();

    // DEBUG4LN("GPS: ", telly.telescope.astro.getLatDec(), telly.telescope.astro.getLongDec(), DEC);
    // telly.telescope.astro.setCurrentTime(1659684268); // 5:36:59 appx
    // DEBUG4LN("Time: ", telly.telescope.astro.getCurrentTime(), telly.telescope.astro.getLocalSiderealTime(), DEC);
    // telly.telescope.astro.setCurrentTime(1659684268 + 3600); // 5:36:59 appx
    // DEBUG4LN("Time: ", telly.telescope.astro.getCurrentTime(), telly.telescope.astro.getLocalSiderealTime(), DEC);
    // telly.telescope.astro.setCurrentTime(1659684268 + 1); // 5:36:59 appx
    // DEBUG4LN("Time: ", telly.telescope.astro.getCurrentTime(), telly.telescope.astro.getLocalSiderealTime(), DEC);
    // // DEBUG2LN(telly.telescope.astro.getCurrentTime(), DEC);
    // // delay(2000);
    // // DEBUG2LN(telly.telescope.astro.getCurrentTime(), DEC);
}

void loop()
{
    static unsigned long lastFastInterval = 0;
    static unsigned long lastSlowInterval = 0;

    unsigned long lastTime = millis();
    if (lastFastInterval != lastTime)
    {
        lastFastInterval = lastTime;
        chassis.tick();
    }

    lastTime /= 10;
    // lastTime /= 500;
    if (lastSlowInterval != lastTime)
    {
        lastSlowInterval = lastTime;
        // blinking led
        digitalWrite(LED_BUILTIN, lastTime & 0x100 ? 0 : 1);
        telly.tick();
    }
}

// void debugfn()
// {
//     unsigned long getCurrentTime = 1659684268;
//     double decLong = 19.04;
//     {
//         unsigned long obstime = getCurrentTime;
//         float64_t d, t, GMST_s, LMST_s;

//         d = fp64_add(fp64_div(fp64_int32_to_float64(obstime), fp64_int32_to_float64(86400)), fp64_sd(2440587.5 - 2451545.0));
//         t = fp64_div(d, fp64_int32_to_float64(36525));

//         DEBUG4LN("D,T:", fp64_ds(d), fp64_ds(t), DEC);

//         // GMST_s = 24110.54841 + 8640184.812866 * t + 0.093104 * pow(t, 2) - 0.0000062 * pow(t, 3);
//         GMST_s = fp64_add(fp64_add(fp64_sd(24110.54841),
//                                    fp64_mul(fp64_sd(8640184.812866), t)),
//                           fp64_sub(fp64_mul(fp64_sd(0.093104), fp64_pow(t, fp64_sd(2.))),
//                                    fp64_mul(fp64_sd(0.0000062), fp64_pow(t, fp64_sd(3.)))));
//         /* convert from UT1=0 */
//         GMST_s = fp64_add(GMST_s, fp64_int32_to_float64(obstime));
//         GMST_s = fp64_sub(GMST_s, fp64_mul(fp64_int32_to_float64(86400), fp64_floor(fp64_div(GMST_s, fp64_int32_to_float64(86400)))));

//         /* adjust to LMST */
//         LMST_s = fp64_add(GMST_s, fp64_mul(fp64_int32_to_float64(3600 / 15), fp64_sd(decLong)));

//         if (LMST_s <= 0)
//         { /* LMST is between 0 and 24h */
//             LMST_s = fp64_add(LMST_s, fp64_int32_to_float64(86400));
//         }

//         // DEBUG3(" sid2 ", LMST_s / 3600., 6);
//         DEBUG3LN("Return FP:", fp64_ds(fp64_div(LMST_s, fp64_int32_to_float64(3600))), DEC);
//     }
//     // old
//     {
//         unsigned long long obstime = getCurrentTime;
//         double d, t, GMST_s, LMST_s;

//         d = (obstime / 86400.0) + 2440587.5 - 2451545.0;
//         t = d / 36525.;

//         DEBUG4LN("D,T:", d, t, DEC);

//         // GMST_s = 24110.54841 + 8640184.812866 * t + 0.093104 * pow(t, 2) - 0.0000062 * pow(t, 3);
//         GMST_s = 24110.54841 + 8640184.812866 * t + 0.093104 * t * t - 0.0000062 * t * t * t;
//         /* convert from UT1=0 */
//         GMST_s += obstime;
//         GMST_s = GMST_s - 86400.0 * floor(GMST_s / 86400.0);

//         /* adjust to LMST */
//         LMST_s = GMST_s + 3600. * decLong / 15.;

//         if (LMST_s <= 0)
//         { /* LMST is between 0 and 24h */
//             LMST_s += 86400.0;
//         }

//         // DEBUG3(" sid2 ", LMST_s / 3600., 6);
//         // return LMST_s / 3600.;
//         DEBUG3LN("Return trad:", LMST_s / 3600., DEC);
//     }
// }