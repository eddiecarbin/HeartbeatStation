#define USE_ARDUINO_INTERRUPTS true
#include <Arduino.h>
#include "SoftwareSerial.h"
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <SPI.h>
#include "Fsm.h"
//#include "BPMController.h"
#include "../lib/SoundPlayer/SoundPlayer.h"
#include <CapacitiveSensor.h>
#include "CapacitiveTouch.h"

#define USE_ARDUINO_INTERRUPTS true
#define GO_TO_WAIT_FOR_USER 2
#define GO_TO_CALCULATE_HEARTBEAT 3
#define GO_TO_DISPLAY_BPM 4

#define FRAME_PER_SECOND 1000 / 60
#define THINK_DELAY 200
#define TOTAL_DIGITS 4

#define BLINK_INTERVAL_0 600
#define BLINK_INTERVAL_1 300

#define BPM_MIN 60
#define BPM_MAX 75

Adafruit_7segment matrix = Adafruit_7segment();
State StateDoNothing(NULL, NULL, NULL);
Fsm fsm(&StateDoNothing);

unsigned long _pause = 0;
int _aniCount = 0;
int digitArray[4] = {0, 1, 3, 4};
bool blinkState = false;
int BPM = 0;
bool userRelased = false;

SoundPlayer soundPlayer(0);
CapacitiveTouch sensor;

long map2(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

void OnIdleWaitForUserEnter()
{

  matrix.clear();
  matrix.writeDisplay();
  _pause = 0;
  BPM = 0;
}

void OnIdleWaitForUserUpdate()
{

  unsigned long m = millis();
  matrix.clear();

  if (m > _pause)
  {
    _pause = m + BLINK_INTERVAL_0;

    blinkState = !blinkState;
    matrix.clear();
    if (blinkState)
    {
      matrix.print(0);
    }
    matrix.writeDisplay();
  }

  if (sensor.detectContact())
  {
    fsm.trigger(GO_TO_CALCULATE_HEARTBEAT);
  }
}

/*------------------------------------------------
/  Display User State
*/

void OnCalculateBPMEnter()
{
  matrix.clear();
  matrix.writeDisplay();
  soundPlayer.PlaySound(1);
}

void OnCalculateBPMUpdate()
{

  if (millis() > _pause)
  {

    matrix.clear();

    matrix.writeDigitRaw(digitArray[_aniCount], 0x40);
    matrix.writeDisplay();
    _aniCount += 1;

    if (_aniCount >= TOTAL_DIGITS)
    {
      _aniCount = 0;
    }

    _pause = millis() + THINK_DELAY;
  }

  if (!sensor.detectContact())
  {
    fsm.trigger(GO_TO_WAIT_FOR_USER);
  }
}

void OnCalculateBPMExit()
{

  BPM = random(BPM_MIN, BPM_MAX);

  soundPlayer.StopSound();
}

void OnDisplayBPMEnter()
{
  userRelased = false;
  matrix.clear();

  matrix.print(BPM);
  matrix.writeDisplay();
}

void OnDisplyBPMUpdate()
{

  if (!sensor.detectContact())
  {
    if (!userRelased)
      userRelased = true;
  }
  else
  {
    if (userRelased)
      fsm.trigger(GO_TO_CALCULATE_HEARTBEAT);
  }
}

State StateWaitForUser(&OnIdleWaitForUserEnter, &OnIdleWaitForUserUpdate, NULL);
State StateCalculateBPM(&OnCalculateBPMEnter, &OnCalculateBPMUpdate, &OnCalculateBPMExit);
State StateDisplayBPM(&OnDisplayBPMEnter, &OnDisplyBPMUpdate, NULL);

void setup()
{

  Serial.begin(9600);
  delay(3000);

  soundPlayer.initialize();

  sensor.intialize();

  fsm.add_transition(&StateWaitForUser, &StateCalculateBPM,
                     GO_TO_CALCULATE_HEARTBEAT, NULL);
  // Demo Start
  //fsm.add_timed_transition(&StateWaitForUser, &StateCalculateBPM, 10000, NULL);

  fsm.add_transition(&StateCalculateBPM, &StateWaitForUser,
                     GO_TO_WAIT_FOR_USER, NULL);

  fsm.add_transition(&StateCalculateBPM, &StateDisplayBPM, GO_TO_DISPLAY_BPM, NULL);

  fsm.add_timed_transition(&StateCalculateBPM, &StateDisplayBPM, 3000, NULL);

  fsm.add_transition(&StateDisplayBPM, &StateCalculateBPM, GO_TO_CALCULATE_HEARTBEAT, NULL);
  fsm.add_timed_transition(&StateDisplayBPM, &StateWaitForUser, 5000, NULL);

  matrix.begin(0x72);
  fsm.goToState(&StateWaitForUser);
}

void loop()
{

  soundPlayer.update();
  fsm.run_machine();

  sensor.update();
  delay(FRAME_PER_SECOND);
}
