#pragma once

#include <Arduino.h>
#include "chassis.h"
#include "astro.h"
#include "nexstar.h"
#include "handwheel.h"
#include "telescopectrl.h"

enum TellyState
{
    Uninitialized,
    Normal,
    Calibration,
    MovingToNorthPole,
    MovingToTop,
    Spiral00,
    Spiral01,
    Spiral11,
    Spiral10,
};

class TellyMain
{
public:
    double spiralFactor = 1.;

    TellyState actualState = Uninitialized;

    MyChassis &chassis;
    MyAstro astro;
    TelescopeControl telescope;
    Handwheel &handwheel;
    NexstarProtocol protocol;

public:
    TellyMain(MyChassis &_chassis, Handwheel &_hwheel) : chassis(_chassis), telescope(astro, _chassis), handwheel(_hwheel), protocol(telescope, astro, Serial3)
    {
    }

    void stopMotion();

    void startSpiral();

#ifdef __GPS
    bool gpsFix = false;
#endif

    void initializeTelescope();
    void slowTick();

    void setup();
};
