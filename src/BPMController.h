#ifndef BPMCONTROLLER_H_
#define BPMCONTROLLER_H_
#include <Arduino.h>
#include "../lib/Smoothed/src/Smoothed.h"
// #include <Filter.h>

#define BPM_MIN 60
#define BPM_MAX 75
#define MIN_BEAT_TIME 1 * 200
//
//https://github.com/sparkfun/AD8232_Heart_Rate_Monitor/blob/master/Software/Heart_Rate_Display_Arduino/Heart_Rate_Display_Arduino.ino
//https://github.com/WorldFamousElectronics/PulseSensorPlayground.git
//https://www.megunolink.com/articles/coding/3-methods-filter-noisy-arduino-measurements/
//https://learn.sparkfun.com/tutorials/ad8232-heart-rate-monitor-hookup-guide?_ga=2.75787026.640950831.1580583873-1651883634.1576708500&_gac=1.50379227.1576708501.Cj0KCQiAuefvBRDXARIsAFEOQ9GSRcsCZ65jw_e34U9y-PJbUBenXqlBOMSiQ3fS8zgCVHwlTXkeJHIaAps2EALw_wcB
//

class BPMController
{
private:
    int output;
    int LOpinp;
    int LOpinn;
    int _BPM;
    bool detectBeat;
    unsigned long lastBeatTime;
    Smoothed<float> mySensor;
    // Smoothed<float> mySensor2;

public:
    BPMController(int out, int pinLOp, int pinLOn);
    ~BPMController();
    void update(void);
    void intialize(void);
    bool detectContact();

    int getBPM();
};

BPMController::BPMController(int out, int pinLOp, int pinLOn)
{
    this->output = out;
    this->LOpinp = pinLOp;
    this->LOpinn = pinLOn;

    pinMode(LOpinp, INPUT); // Setup for leads off detection LO +
    pinMode(LOpinn, INPUT); // Setup for leads off detection LO -
}

int BPMController::getBPM()
{

    _BPM = random(BPM_MIN, BPM_MAX);

    return _BPM;
}
bool BPMController::detectContact()
{
    return detectBeat;
}

void BPMController::intialize()
{
    detectBeat = false;
    mySensor.begin(SMOOTHED_AVERAGE, 10);

    // Initialise the second sensor value store. We want this one to be a simple linear recursive exponential filter.
    // We set the filter level to 10. Higher numbers will result in less filtering/smoothing. Lower number result in more filtering/smoothing
    // mySensor2.begin(SMOOTHED_EXPONENTIAL, 10);
}

void BPMController::update()
{
    unsigned long m = millis();
    if ((digitalRead(LOpinp) == 1) || (digitalRead(LOpinn) == 1))
    {
        detectBeat = false;
        lastBeatTime = m;
        Serial.println(":what");
    }
    else
    {
        // send the value of analog input 0:

        if (m > (lastBeatTime + MIN_BEAT_TIME))
        {
        }

        detectBeat = true;
        int sensorOutput = analogRead(output);
        Serial.println(sensorOutput);
        //Serial.print(",");
        // FilteredSignal->Filter(sensorOutput);
        //Serial.println(FilteredSignal->Current());
        //mySensor.add(sensorOutput);
        // mySensor2.add(sensorOutput);
        //float s = mySensor.get();
        //Serial.println(mySensor.get());
        // Serial.print(",");
        // Serial.println(mySensor2.get());
        if (sensorOutput < 200)
        {

            detectBeat = false;
        }
    }
}
BPMController::~BPMController()
{
}

#endif /* BPMCONTROLLER_H_ */