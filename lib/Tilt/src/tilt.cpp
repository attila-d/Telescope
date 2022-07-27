#include "tilt.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
bool accelInitied = false;

void initADXL345()
{
    if (accel.begin())
    {
        accel.setRange(ADXL345_RANGE_2_G);
        accelInitied = true;
    }
    else
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        if (Serial)
        {
            Serial.println("Ooops, no ADXL345 detecte");
        }
        // while (1)
        //     ;
    }
}

sensors_event_t event;

float getTilt()
{
    if (accelInitied)
    {
        accel.getEvent(&event);
        return (float)(atan2(event.acceleration.y, event.acceleration.x) * 180 / PI);
    }
    return 45;
}
