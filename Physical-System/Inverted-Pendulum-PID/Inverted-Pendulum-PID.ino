#include <EnableInterrupt.h>          
#include <digitalWriteFast.h>  
#include "global.h"       
#include "motor.h"
#include "pendulum.h"

#define encoderA 2
#define encoderB 3
#define motorEncoder 4

int encoderDir = 0;  
volatile int motorCount = 0; 

void setup() {
  motor.SETUP();
  pendulum.SETUP(); 
  Serial.begin(9600);

  pinMode(encoderA, INPUT);
  pinModeFast(encoderB, INPUT);
  pinMode(motorEncoder, INPUT); 
  enableInterrupt(encoderA, pulse, RISING);
  enableInterrupt(motorEncoder, motorPulse, RISING);

  //pendulum.CALIBRATE(); 
}

void loop() {
  Serial.print(pendulum.ANGLE());
  Serial.print('\t');
  Serial.println();  
  pendulum.CHECKLIMITS();         // do not need to be actively monitored via interrupts because of slow cart speed relative to loop() cycle speed
  delay(50);
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