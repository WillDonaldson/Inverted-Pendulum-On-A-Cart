#include <EnableInterrupt.h>    // one interrupt is on Pin 4 - if connected to pin 3 then this library isn't needed. 
#include <digitalWriteFast.h>  
#include <PID_v1.h>
#include "global.h"       
#include "motor.h"
#include "pendulum.h"

#define encoderA 2
#define encoderB 3
#define motorEncoder 4
#define pot1 A0           //for tuning PID coefficients
#define pot2 A1           //for tuning PID coefficients
#define pot3 A2           //for tuning PID coefficients
#define currentInput A5          

int encoderDir = 0;  

// ------ PID variables ------
double Setpoint = 0.0;
double Kp = 0.0;
double Ki = 0.0;
double Kd = 0.0;
double Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//int prevEncDir = 0;           //uncomment when tuning PID parameters
//unsigned long prevOscT = 0;   //uncomment when tuning PID parameters

int t = 0;
int t2 = 0;
int pCount = 0;
int prevEncDir =0;
unsigned long prevOscT = 0;

void setup() {
  motor.SETUP();
  pendulum.SETUP(); 
  Serial.begin(9600);
  
  pinMode(encoderA, INPUT_PULLUP);
  pinModeFast(encoderB, INPUT_PULLUP);  
  pinMode(motorEncoder, INPUT_PULLUP);   
  enableInterrupt(encoderA, pulse, RISING);
  enableInterrupt(motorEncoder, motorPulse, RISING);

  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);
  pinMode(pot3, INPUT);
  pinMode(currentInput, INPUT);
  pendulum.CALIBRATE(); 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(30);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetTunings(Kp, Ki, Kd);
}

void loop() {
  Input = pendulum.ANGLE();
  Kp = analogRead(pot1)*10.0;  //optimal range narrowed to about [3800, 4500]
  Ki = analogRead(pot2)*30.0;
  Kd = analogRead(pot3)*2;
  
  
  myPID.SetTunings(Kp, Ki, Kd);
  Input = pendulum.ANGLE();
  myPID.Compute();

  
  if(abs(Input) < 0.4){
    //myPID.Compute();
    
    //g_motorDir = Output/abs(Output);
    int speed = abs(Output);
    if(speed < 40){ speed = 0; g_motorDir =0; }      // prevent motor stalls
    motor.DRIVEMOTOR(g_motorDir, speed); 
  }
  else{
    //g_motorDir = 0;
    motor.DRIVEMOTOR(g_motorDir, 0);
    // hard reset of PID variables
    // don't want to accumulate integral term when pendulum is hanging downwards
    myPID.SetOutputLimits(0.0, 1.0);  
    myPID.SetOutputLimits(-1.0, 0.0);  
    myPID.SetOutputLimits(-255, 255);  
    // this should be before if(abs(Input) < 0.4) and myPID.compute() - fix later
  }

  if(abs(Input) > (PI-0.4)){
    // recalibrate encoder if it has been hanging downwards for more than 1 sec
    if((millis() - t2) > 3000){
      t2=millis();
      if(pCount == g_pendulumEncA){
        Serial.println("Recalibrate");
        g_pendulumEncA = 512;  
        
      }  
      pCount = g_pendulumEncA; 
    } 
  }
  
  if((millis() - t) > 100){
    Serial.print("Kp: ");
    Serial.print(Kp);
    Serial.print("\t");
    Serial.print("Ki: ");
    Serial.print(Ki);
    Serial.print("\t");
    Serial.print("Kd: ");
    Serial.print(Kd);
    Serial.print("\t");
    Serial.print("Angle: ");
    Serial.print(Input);
    Serial.print("\t");
    Serial.print("g_motorEnc: ");
    Serial.print(g_motorEnc);
    Serial.print("\t");
    Serial.print("Output: ");
    Serial.print(Output);
    t = millis();
    Serial.print("\n");
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
  encoderDir= ((bool) digitalReadFast(encoderB) ==  HIGH) ? 1 : -1;
  if(prevEncDir != encoderDir){
    // used to find period of ultimate gain (uncomment while tuning PID coefficients)
    unsigned long currOscT = millis();
    unsigned long halfPeriod = currOscT - prevOscT;
    prevOscT = currOscT;
    Serial.print(halfPeriod);  //bad practice to use print() in ISR - fix later
    Serial.println();
  }
  prevEncDir = encoderDir;
  */
}