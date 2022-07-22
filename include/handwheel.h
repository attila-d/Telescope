#pragma once

#include <RotaryEncoder.h>
#include <OneButton.h>

extern bool attachHandwheel;
extern long stepSize;

class Handwheel
{
    RotaryEncoder &encAlt;
    RotaryEncoder &encAzi;

    OneButton &btnAlt;
    OneButton &btnAzi;

public:
    int initialAziButton;
    int initialAltButton;

    bool isAziPressed();
    bool isAltPressed();
public:
    Handwheel(RotaryEncoder &_encAlt, RotaryEncoder &_encAzi, OneButton &_btnAlt, OneButton &_btnAzi)
        : encAlt(_encAlt), encAzi(_encAzi), btnAlt(_btnAlt), btnAzi(_btnAzi)
    {
    }

    void setup();
    void tick();
};
