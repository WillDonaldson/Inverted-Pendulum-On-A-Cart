#ifndef motor_h
#define motor_h

#include "Arduino.h"

class mclass
{
    public:
        mclass();
        void SETUP();  
        void DRIVEMOTOR(int Direction, int Speed);
    private:
};

extern mclass motor;

#endif