#include <Arduino.h>
#include <unity.h>

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

extern TellyMain telly;

void setUp()
{
}

void tearDown()
{
}

void test_led_builtin_pin_number(void)
{
  TEST_ASSERT_EQUAL(13, LED_BUILTIN);
}

void test_led_state_high(void)
{
  digitalWrite(LED_BUILTIN, HIGH);
  TEST_ASSERT_EQUAL(HIGH, digitalRead(LED_BUILTIN));
}

void test_led_state_low(void)
{
  digitalWrite(LED_BUILTIN, LOW);
  TEST_ASSERT_EQUAL(LOW, digitalRead(LED_BUILTIN));
}

// int main( int argc, char ** argv ) {
//   UNITY_BEGIN();
//   UNITY_END();
// }

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);

  telly.setup();

  pinMode(LED_BUILTIN, OUTPUT);

  UNITY_BEGIN(); // IMPORTANT LINE!
  RUN_TEST(test_led_builtin_pin_number);
}

uint8_t i = 0;
uint8_t max_blinks = 5;

void loop()
{
  if (i < max_blinks)
  {
    RUN_TEST(test_led_state_high);
    delay(500);
    RUN_TEST(test_led_state_low);
    delay(500);
    i++;
  }
  else if (i == max_blinks)
  {
    UNITY_END(); // stop unit testing
  }
}