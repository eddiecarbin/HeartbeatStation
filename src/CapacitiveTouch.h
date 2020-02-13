#ifndef CAPACITIVETOUCH_H_
#define CAPACITIVETOUCH_H_
#include <CapacitiveSensor.h>
CapacitiveSensor cs(10, 11);

#define bounceTime 500

class CapacitiveTouch
{
private:
    bool detectBeat;
    long bounceDelay;
    bool bounceState;

public:
    CapacitiveTouch(/* args */);
    ~CapacitiveTouch();

    void update(void);
    void intialize(void);
    bool detectContact();
};
bool CapacitiveTouch::detectContact()
{
    return detectBeat;
}

void CapacitiveTouch::intialize()
{
    detectBeat = false;
    bounceState = false;
    bounceDelay = 0;
}

void CapacitiveTouch::update()
{
    long value = cs.capacitiveSensor(30);

    long m = millis();

    // Serial.println(value);
    if (value == 0)
    {
        if (bounceState == false)
        {
            bounceState = true;
            bounceDelay = m + bounceTime;
        }

        if (m > bounceDelay)
        {
            detectBeat = true;
        }
    }
    else
    {
        if (bounceState == true)
        {
            bounceState = false;
            bounceDelay = m + bounceTime;
        }
        if (m > bounceDelay)
        {
            detectBeat = false;
        }
    }
}

CapacitiveTouch::CapacitiveTouch(/* args */)
{
}

CapacitiveTouch::~CapacitiveTouch()
{
}

#endif /* CAPACITIVETOUCH_H_ */