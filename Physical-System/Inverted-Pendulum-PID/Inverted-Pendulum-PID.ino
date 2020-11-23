#include <EnableInterrupt.h>          
#include <digitalWriteFast.h>  
#include <PID_v1.h>
#include "global.h"       
#include "motor.h"
#include "pendulum.h"

#define encoderA 2
#define encoderB 3
#define motorEncoder 4
#define pot1 A0           //for tuning PID coefficients

int encoderDir = 0;  

// ------ PID variables ------
double Setpoint = 0.0;
double Kp = 3000.0;
double Ki = 30000.0;
double Kd = 75.0;
double Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//int prevEncDir = 0;           //uncomment when tuning PID parameters
//unsigned long prevOscT = 0;   //uncomment when tuning PID parameters

void setup() {
  motor.SETUP();
  pendulum.SETUP(); 
  Serial.begin(9600);

  pinMode(encoderA, INPUT);
  pinModeFast(encoderB, INPUT);
  pinMode(motorEncoder, INPUT); 
  enableInterrupt(encoderA, pulse, RISING);
  enableInterrupt(motorEncoder, motorPulse, RISING);

  pinMode(pot1, INPUT);
  pendulum.CALIBRATE(); 

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(30);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetTunings(Kp, Ki, Kd);
}

void loop() {
  Input = pendulum.ANGLE();
  if(abs(Input) < (PI/4.)){
    myPID.Compute();
    int dir = Output/abs(Output);
    int speed = abs(Output);
    if(speed < 40){ speed = 0; dir =0; }      // prevent motor stalls
    motor.DRIVEMOTOR(dir, speed); 
  }
  else{
    motor.DRIVEMOTOR(0, 0);
  }
  pendulum.CHECKLIMITS();         // do not need to be actively monitored via interrupts because of slow cart speed relative to loop() cycle speed
}

//----------- ISR -----------

void motorPulse(){
  // since motor encoder is not quadrature the position value will be inaccurate
  // due to inertial overshoot when changing direction/stopping
  g_motorEnc += g_motorDir;
}

void pulse(){  
  g_pendulumEncA += ((bool) digitalReadFast(encoderB) ==  HIGH) ? 1 : -1;
  /*
  if(prevEncDir != encoderDir){
    // used to find period of ultimate gain (uncomment while tuning PID coefficients)
    unsigned long currOscT = millis();
    unsigned long halfPeriod = currOscT - prevOscT;
    prevOscT = currOscT;
    Serial.println(halfPeriod);  //bad practice to use print() in ISR - fix later
  }
  prevEncDir = encoderDir;
  */
}