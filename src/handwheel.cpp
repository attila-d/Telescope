#include "chassis.h"
#include "handwheel.h"
#include "main.h"

extern RotaryEncoder encAlt;
extern RotaryEncoder encAzi;

extern AccelStepper stepAlt;
extern AccelStepper stepAzi;

bool attachHandwheel = false; // if true, hwheel moves position too, if not, chassis moves but telescope point is compensated

long altPrev = 0;
long aziPrev = 0;
long stepSize = 256;

// extern TellyMain telly;
extern MyChassis chassis;

bool Handwheel::isAziPressed()
{
    return digitalRead(AZI_BTN_PIN);
}
bool Handwheel::isAltPressed()
{
    return digitalRead(ALT_BTN_PIN);
}

void Handwheel::setup()
{
    // btnAzi.tick();
    // btnAlt.tick();

    // initialAziButton = btnAzi.getPressedTicks() > 0;
    // initialAltButton = btnAlt.getPressedTicks() > 0;

    initialAziButton = isAziPressed();
    initialAltButton = isAltPressed();
}

void Handwheel::tick()
{
    encAlt.tick();
    encAzi.tick();
    btnAzi.tick();
    btnAlt.tick();

    if (digitalRead(AZI_BTN_PIN) == 0)
    {
        long altPos = -encAlt.getPosition();
        if (altPos != altPrev)
        {
            if (altPos < altPrev)
            {
                stepSize >>= 1;
            }
            else
            {
                stepSize <<= 1;
            }
            if (stepSize > 8192)
            {
                stepSize = 8192;
            }
            if (stepSize < 1)
            {
                stepSize = 1;
            }
            DEBUG3LN("Step size = ", stepSize, DEC);
            altPrev = altPos;
        }
    }
    else
    {
        long alt0 = stepAlt.currentPosition();
        long azi0 = stepAzi.currentPosition();

        long altPos = -encAlt.getPosition();
        if (altPos != altPrev)
        {
            alt0 += (altPos - altPrev) * stepSize;
            altPrev = altPos;
            // DEBUG3LN("ALT pos:",alt0, DEC);
        }
        long aziPos = encAzi.getPosition();
        if (aziPos != aziPrev)
        {
            azi0 += (aziPos - aziPrev) * stepSize;
            aziPrev = aziPos;
            // DEBUG3LN("AZI pos:",azi0, DEC);
        }

        // Serial.println("HW tick");
        // Serial.flush();
        if (stepAlt.currentPosition() != alt0 || stepAzi.currentPosition() != azi0)
        {
            // DEBUG4("HandWheel change=", alt0, azi0, DEC);
            if (!attachHandwheel)
            {
                chassis.altStepOffset += (alt0 - stepAlt.targetPosition());
                chassis.aziStepOffset += (azi0 - stepAzi.targetPosition());
                // DEBUG3("MyChassis offset to:", altStepOffset, DEC);
                // DEBUG3LN(":", aziStepOffset, DEC);
            }
            chassis.gotoSteps(alt0, azi0);
        }
    }
}