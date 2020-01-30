
#include <Arduino.h>
#include "SoftwareSerial.h"
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <SPI.h>
#include "Fsm.h"
//#include <PulseSensorPlayground.h>
#include <SoftwareSerial.h>

#define USE_ARDUINO_INTERRUPTS true
#define GO_TO_WAIT_FOR_USER 2
#define GO_TO_CALCULATE_HEARTBEAT 3
#define GO_TO_DISPLAY_BPM 4

#define FRAME_PER_SECOND 60
#define THINK_DELAY 200
#define TOTAL_DIGITS 4

#define BPM_MIN 60
#define BPM_MAX 75

#define BLINK_INTERVAL_0 600
#define BLINK_INTERVAL_1 300

Adafruit_7segment matrix = Adafruit_7segment();
State StateDoNothing(NULL, NULL, NULL);
Fsm fsm(&StateDoNothing);
//PulseSensorPlayground pulseSensor;

int Signal;          // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 550; // Determine which Signal to "count as a beat", and which to ingore.
unsigned long _pause = 0;
int _aniCount = 0;
int digitArray[4] = {0, 1, 3, 4};
bool blinkState = false;
int BPM = 0;

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

  if ((digitalRead(10) == 1) || (digitalRead(11) == 1))
  {
    //Serial.println(0);
  }
  else
  {
    // send the value of analog input 0:
    //Serial.println(analogRead(A0));
  }
}

/*------------------------------------------------
/  Display User State
*/

void OnCalculateBPMEnter()
{
  matrix.clear();
  matrix.writeDisplay();
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

  // FAILED To get heart beat output a random number  of average heartbeat
  BPM = random(BPM_MIN, BPM_MAX);
}

void OnDisplayBPMEnter()
{
  matrix.clear();

  matrix.print(BPM);
  matrix.writeDisplay();
}

void OnDisplyBPMUpdate()
{
}

State StateWaitForUser(&OnIdleWaitForUserEnter, &OnIdleWaitForUserUpdate, NULL);
State StateCalculateBPM(&OnCalculateBPMEnter, &OnCalculateBPMUpdate, NULL);
State StateDisplayBPM(&OnDisplayBPMEnter, &OnDisplyBPMUpdate, NULL);


//https://www.megunolink.com/articles/coding/3-methods-filter-noisy-arduino-measurements/


void setup()
{

  Serial.begin(9600);

  delay(3000);

  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -

  fsm.add_transition(&StateWaitForUser, &StateCalculateBPM,
                     GO_TO_CALCULATE_HEARTBEAT, NULL);
  fsm.add_timed_transition(&StateWaitForUser, &StateCalculateBPM, 5000, NULL);
  
  fsm.add_transition(&StateCalculateBPM, &StateWaitForUser,
                     GO_TO_WAIT_FOR_USER, NULL);

  fsm.add_transition(&StateCalculateBPM, &StateDisplayBPM, GO_TO_DISPLAY_BPM, NULL);

  fsm.add_timed_transition(&StateCalculateBPM, &StateDisplayBPM, 7000, NULL);

  fsm.add_transition(&StateDisplayBPM, &StateCalculateBPM, GO_TO_CALCULATE_HEARTBEAT, NULL);
  fsm.add_timed_transition(&StateDisplayBPM, &StateWaitForUser, 10000, NULL);

  matrix.begin(0x72);

  // matrix.print(0);
  // matrix.writeDigitRaw(1, 0x40);
  //matrix.drawColon(true);
  //matrix.writeDisplay();

  fsm.goToState(&StateWaitForUser);
}

void loop()
{
  fsm.run_machine();
  delay(1);
}
