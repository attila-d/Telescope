
#include "chassis.h"

#include "debug.h"

void MyChassis::gotoSteps(long altStep, long aziStep) {
    DEBUG3("goToSteps=", altStep, DEC);
    DEBUG3LN(":", aziStep, DEC);
    if (stepAlt.currentPosition() != altStep) {
        DEBUG3("MyChassis Moving ALT:", stepAlt.currentPosition(), DEC);
        DEBUG3LN(" to ", altStep, DEC);
        stepAlt.moveTo(altStep);
    }
    long aziCurr = stepAzi.currentPosition() % (aziMax - aziMin);
    if (aziCurr != (aziStep % (aziMax - aziMin))) {
        DEBUG3("MyChassis Moving AZI:", aziCurr, DEC);
        DEBUG3LN(" to ", aziStep, DEC);
        if (abs(aziStep - aziCurr) > (aziMax - aziMin) / 2) {  // more than half round
            if (aziStep > aziCurr) {
                aziStep -= (aziMax - aziMin);
                // DEBUGLN("Correction down");
            } else {
                aziStep += (aziMax - aziMin);
                // DEBUGLN("Correction up");
            }
        }

        stepAzi.moveTo(aziStep);
    }
}

void MyChassis::gotoAltAzi(double altDeg, double aziDeg) {
    long lAzi = calcAziSteps(aziDeg);
    long lAlt = calcAltSteps(altDeg);
    DEBUG3("Chassis is going to:", altDeg, 6);
    DEBUG3(":", aziDeg, 6);
    DEBUG3("->", lAlt, DEC);
    DEBUG3LN(":", lAzi, DEC);
    gotoSteps(lAlt, lAzi);
}

void MyChassis::syncAltAzi(double altDeg, double aziDeg) {
    DEBUG4LN("Chassis is syncing AltAzi:", altDeg, aziDeg, 6);
    // reverse calculate offset from actual position and calculated alt/azimuth
    long lAzi = calcAziSteps(aziDeg) - aziStepOffset;  // we need to subtract, because calcXX implicitly adds
    long lAlt = calcAltSteps(altDeg) - altStepOffset;
    altStepOffset = stepAlt.currentPosition() - lAlt;
    aziStepOffset = stepAzi.currentPosition() - lAzi;
    DEBUG4LN("->", altStepOffset, aziStepOffset, DEC);
}

