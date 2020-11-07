#ifndef pendulum_h
#define pendulum_h

#include "Arduino.h"

class pclass
{
    public:
        pclass();
        void SETUP();
        float ANGLE();
        void CALIBRATE();
        bool CHECKLIMITS();
    private:
};

extern pclass pendulum;

#endif