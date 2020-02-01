#define USE_ARDUINO_INTERRUPTS true
#include <Arduino.h>
#include "SoftwareSerial.h"
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <SPI.h>
#include "Fsm.h"
#include "../lib/PulseSensorPlayground/PulseSensorPlayground.h"
#include <SoftwareSerial.h>
// #include "../lib/HeartSpeed/HeartSpeed.h"
#include <Filter.h>

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
int PulseWire = 0;
bool userRelased = false;

PulseSensorPlayground pulseSensor; // Creates an instance of the PulseSensorPlayground object called "pulseSensor"

ExponentialFilter<long> ADCFilter(5, 0);

//https://github.com/sparkfun/AD8232_Heart_Rate_Monitor/blob/master/Software/Heart_Rate_Display_Arduino/Heart_Rate_Display_Arduino.ino
//https://github.com/WorldFamousElectronics/PulseSensorPlayground.git
//https://www.megunolink.com/articles/coding/3-methods-filter-noisy-arduino-measurements/
//

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

  if ((digitalRead(10) == 1) || (digitalRead(11) == 1))
  {
    Serial.println('!');
  }
  else
  {
    // send the value of analog input 0:

    int sensorOutput = analogRead(A0);

    Serial.println(analogRead(A0));

    if (sensorOutput > 100)
    {
      fsm.trigger(GO_TO_CALCULATE_HEARTBEAT);
    }
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

  if ((digitalRead(10) == 1) || (digitalRead(11) == 1))
  {
    // Serial.println('!');
    fsm.trigger(GO_TO_WAIT_FOR_USER);
  }
  else
  {
    // send the value of analog input 0:

    // int sensorOutput = analogRead(A0);
  }
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
  if ((digitalRead(10) == 1) || (digitalRead(11) == 1))
  {
    // Serial.println('!');

    if (!userRelased)
      userRelased = true;
  }
  else
  {
    // send the value of analog input 0:
    if (userRelased)
      fsm.trigger(GO_TO_CALCULATE_HEARTBEAT);

    // int sensorOutput = analogRead(A0);
  }
}

State StateWaitForUser(&OnIdleWaitForUserEnter, &OnIdleWaitForUserUpdate, NULL);
State StateCalculateBPM(&OnCalculateBPMEnter, &OnCalculateBPMUpdate, NULL);
State StateDisplayBPM(&OnDisplayBPMEnter, &OnDisplyBPMUpdate, NULL);

void setup()
{

  Serial.begin(9600);
  delay(3000);

  // soundPlayer.initialize();

  // Configure the PulseSensor object, by assigning our variables to it.
  // pulseSensor.analogInput(PulseWire);
  // pulseSensor.setThreshold(Threshold);

  // Double-check the "pulseSensor" object was created and "began" seeing a signal.
  // if (pulseSensor.begin())
  // {
  //   Serial.println("We created a pulseSensor Object !"); //This prints one time at Arduino power-up,  or on Arduino reset.
  // }
  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -

  fsm.add_transition(&StateWaitForUser, &StateCalculateBPM,
                     GO_TO_CALCULATE_HEARTBEAT, NULL);

  fsm.add_transition(&StateCalculateBPM, &StateWaitForUser,
                     GO_TO_WAIT_FOR_USER, NULL);

  fsm.add_transition(&StateCalculateBPM, &StateDisplayBPM, GO_TO_DISPLAY_BPM, NULL);

  fsm.add_timed_transition(&StateCalculateBPM, &StateDisplayBPM, 7000, NULL);

  fsm.add_transition(&StateDisplayBPM, &StateCalculateBPM, GO_TO_CALCULATE_HEARTBEAT, NULL);
  fsm.add_timed_transition(&StateDisplayBPM, &StateWaitForUser, 10000, NULL);

  matrix.begin(0x72);
  fsm.goToState(&StateWaitForUser);
}

void loop()
{


  // int myBPM = pulseSensor.getBeatsPerMinute(); // Calls function on our pulseSensor object that returns BPM as an "int".
  //                                              // "myBPM" hold this BPM value now.

  // if (pulseSensor.sawStartOfBeat())
  // {                                               // Constantly test to see if "a beat happened".
  //   Serial.println("♥  A HeartBeat Happened ! "); // If test is "true", print a message "a heartbeat happened".
  //   Serial.print("BPM: ");                        // Print phrase "BPM: "
  //   Serial.println(myBPM);                        // Print the value inside of myBPM.
  // }

  // pulseSensor.outputSample();

  // if (pulseSensor.sawStartOfBeat()) {
  //   pulseSensor.outputBeat();
  // }

  if ((digitalRead(10) == 1) || (digitalRead(11) == 1))
  {
    Serial.println('!');
  }
  else
  {
    // send the value of analog input 0:
    Serial.println(analogRead(A0));
  }
  //Wait for a bit to keep serial data from saturating

  fsm.run_machine();
  delay(1);
}
