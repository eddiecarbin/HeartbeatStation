
#include <Arduino.h>
#include "SoftwareSerial.h"
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <SPI.h>
#include "Fsm.h"
#include <PulseSensorPlayground.h>
#include <SoftwareSerial.h>

#define USE_ARDUINO_INTERRUPTS true
#define GO_TO_WAIT_FOR_USER 2
#define GO_TO_CALCULATE_HEARTBEAT 3
#define FRAME_PER_SECOND 60

Adafruit_7segment matrix = Adafruit_7segment();
State StateDoNothing(NULL, NULL, NULL);
Fsm fsm(&StateDoNothing);
PulseSensorPlayground pulseSensor;

void OnIdleWaitForUserEnter()
{
}

void OnIdleWaitForUserUpdate()
{
  
}


/*------------------------------------------------
/  Display User State
*/

void OnCalculateWeightEnter()
{
}

void OnCalculateWeightUpdate()
{
 
}

State StateIdleWaitForUser(&OnIdleWaitForUserEnter, &OnIdleWaitForUserUpdate, NULL);
State StateCalculateWeight(&OnCalculateWeightEnter, &OnCalculateWeightUpdate, NULL);

void setup()
{

  Serial.begin(9600);

  delay(3000);

 fsm.add_transition(&StateIdleWaitForUser, &StateCalculateWeight,
                     GO_TO_CALCULATE_HEARTBEAT, NULL);
  fsm.add_transition(&StateCalculateWeight, &StateIdleWaitForUser,
                     GO_TO_WAIT_FOR_USER, NULL);

  matrix.begin(0x72);

  matrix.print(0);
  matrix.writeDigitRaw(1,0x40);
  matrix.drawColon(true);
  matrix.writeDisplay();

  fsm.goToState(&StateIdleWaitForUser);

}

void loop()
{
  fsm.run_machine();

  delay(1000 / FRAME_PER_SECOND);
}
