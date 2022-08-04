#include "main.h"

#include <Arduino.h>

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

#define LED_BUILTIN 13

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

    // telly.telescope.astro.setCurrentTime(123123123);
    // DEBUG2LN(telly.telescope.astro.getCurrentTime(), DEC);
    // delay(2000);
    // DEBUG2LN(telly.telescope.astro.getCurrentTime(), DEC);
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