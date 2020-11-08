#include "Arduino.h"
#include "pendulum.h"
#include "global.h"
#include "motor.h"

const int pulsesPerRev = 1024;      // unique to encoder
const int trackLength = 350;        // number of pulses measured by motor encoder. Found using Measure-Track-Length.ino
int currCount = 0;
int prevCount;

pclass::pclass(){
}

void pclass::SETUP(){
  pinMode(g_leftLimit, INPUT);
  pinMode(g_rightLimit, INPUT);
}

float pclass::ANGLE(){
  // converts encoder data to angle, theta = [-pi, pi]
  // assuming function is sampled frequently (it is) g_pendulumEncA will never suffer integer overflow
  if(abs(g_pendulumEncA) > pulsesPerRev/2){
    int polarity = g_pendulumEncA/abs(g_pendulumEncA); 
    volatile int divisor = g_pendulumEncA/pulsesPerRev;   // will always = 0 (assuming frequent sampling), included for rigor
    volatile int normCount = (g_pendulumEncA-divisor*pulsesPerRev)-(pulsesPerRev*polarity);
    g_pendulumEncA = normCount; 
  }
  float angle = (g_pendulumEncA/1024.)*2*PI;
  return angle;
}

void pclass::CALIBRATE(){
  // move cart to rightmost limit switch  
  while(digitalRead(g_rightLimit) == 0){
    g_motorDir = -1;
    motor.DRIVEMOTOR(g_motorDir, 80);
    delay(1);
  }
  
  // wait for the pendulum to settle
  g_motorDir = 0;
  motor.DRIVEMOTOR(g_motorDir, 0);
  do{
    prevCount = currCount;  
    delay(2500);                   
    currCount = g_pendulumEncA;
  }while(prevCount != currCount);        
  g_pendulumEncA = pulsesPerRev/2;        // calibrate count s.t. theta = 0, pi when the pendulum is vertical upwards and downwards, respectively. 

  // move cart to centre point of track 
  g_motorEnc = 0;
  while(g_motorEnc < (trackLength/2.)){
    g_motorDir = 1;
    motor.DRIVEMOTOR(g_motorDir, 80);
    delay(1);
  }
  g_motorDir = 0;
  motor.DRIVEMOTOR(g_motorDir, 0);
  delay(500);
}

bool pclass::CHECKLIMITS(){
  if((digitalRead(g_leftLimit) == 1) || (digitalRead(g_rightLimit) == 1)){
    g_motorDir = 0;
    motor.DRIVEMOTOR(g_motorDir, 0);
    CALIBRATE();
    return true; 
  } 
  else{
    return false;
  }
}

pclass pendulum = pclass();