#pragma once

#include <AccelStepper.h>
#include <math.h>

#include "util.h"

// how many stepper steps required for a whole round turn? D=251mm, perimeter=788.539, GT2 steps=2/mm, microsteps=16, shaft diameter=16 steps, nema17 200 step/revolution
// 78854
#define STEPS_PER_REVOLUTION ((long)(252 * PI * 16 * 200 / (2 * 16)))
// #define STEPS_PER_REVOLUTION 79110

#define altMin (0)
#define altMax (STEPS_PER_REVOLUTION / 4)
#define aziMin (-STEPS_PER_REVOLUTION / 2)
#define aziMax (STEPS_PER_REVOLUTION / 2)

// chassis interface

class MyChassis
{
    AccelStepper &stepAlt;
    AccelStepper &stepAzi;

protected:
    long calcAziSteps(double aziDeg) { return (long)mapDouble(dRange(aziDeg, -180., 180.), -180., 180., aziMin, aziMax) + aziStepOffset; }
    long calcAltSteps(double altDeg) { return (long)mapDouble(dRange(altDeg, 0., 360.), 0, 90, altMin, altMax) + altStepOffset; }

public:
    MyChassis(AccelStepper &_stepAlt, AccelStepper &_stepAzi) : stepAlt(_stepAlt), stepAzi(_stepAzi)
    {
    }

    void gotoAltAzi(double altDeg, double aziDeg);
    void syncAltAzi(double altDeg, double aziDeg);
    void gotoSteps(long altStep, long aziStep);

    inline bool isGoToInProgress() { return getAltTarget() != getAltCurrent() || getAziTarget() != getAziCurrent(); }

    inline double getAltCurrent() { return mapDouble(lRange(stepAlt.currentPosition() - altStepOffset, altMin, altMax), altMin, altMax, 0., 90.); };
    inline double getAltTarget() { return mapDouble(lRange(stepAlt.targetPosition() - altStepOffset, altMin, altMax), altMin, altMax, 0., 90.); };
    inline double getAziCurrent() { return mapDouble(lRange(stepAzi.currentPosition() - aziStepOffset, aziMin, aziMax), aziMin, aziMax, -180., 180.); };
    inline double getAziTarget() { return mapDouble(lRange(stepAzi.targetPosition() - aziStepOffset, aziMin, aziMax), aziMin, aziMax, -180., 180.); };

    long altStepOffset = 0;
    long aziStepOffset = 0;

    void setup()
    {
        stepAlt.setAcceleration(2500);
        stepAlt.setMaxSpeed(8000);

        stepAzi.setAcceleration(2500);
        stepAzi.setMaxSpeed(8000);

        stepAlt.enableOutputs();
        stepAzi.enableOutputs();
    }

    void tick()
    {
        stepAlt.run();
        stepAzi.run();
    }
};
