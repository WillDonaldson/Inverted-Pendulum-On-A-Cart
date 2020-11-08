// a script for measuring the length of the slide rail track
// units are [pulses] of the motor encoder

#include <EnableInterrupt.h>          
#include <digitalWriteFast.h>  
#include "global.h"       
#include "motor.h"
#include "pendulum.h"

#define encoderA 2
#define encoderB 3
#define motorEncoder 4

int encoderDir = 0;  

void setup() {
  motor.SETUP();
  pendulum.SETUP(); 
  Serial.begin(9600);

  pinMode(encoderA, INPUT);
  pinModeFast(encoderB, INPUT);
  pinMode(motorEncoder, INPUT); 
  enableInterrupt(encoderA, pulse, RISING);
  enableInterrupt(motorEncoder, motorPulse, RISING);

  pendulum.CALIBRATE();

  g_motorEnc = 0;
  while(digitalRead(g_leftLimit) == 0){
      g_motorDir = 1;
      motor.DRIVEMOTOR(g_motorDir, 80);
      delay(1);
  }
  g_motorDir = 0;
  motor.DRIVEMOTOR(g_motorDir, 0);
  Serial.println(g_motorEnc);  
}

void loop(){
}

//----------- ISR -----------

void motorPulse(){
  // since motor encoder is not quadrature the position value will be inaccurate
  // due to inertial overshoot when changing direction/stopping
  motorCount += g_motorDir;
}

void checkDirection(){
  if((bool) digitalReadFast(encoderB) ==  HIGH){                             //digitalReadFast() is faster than digitalRead()
    encoderDir = 1;  
  }
  else{
    encoderDir = -1;
  }
}

void pulse(){  
  checkDirection();
  g_pendulumEncA += encoderDir;
}