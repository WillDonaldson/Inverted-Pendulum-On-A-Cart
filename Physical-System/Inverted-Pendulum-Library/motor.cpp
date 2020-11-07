#include "Arduino.h"
#include "motor.h"

const uint8_t motor1_pin = 5;
const uint8_t motor2_pin = 6;
const uint8_t speed_pin = 9;
const uint8_t motorEncoder = 4;

mclass::mclass(){
}

void mclass::SETUP(){
    pinMode(motor1_pin, OUTPUT);
    pinMode(motor2_pin, OUTPUT);
    pinMode(speed_pin, OUTPUT);
}

void mclass::DRIVEMOTOR(int Direction, int Speed){
  switch (Direction){
    case -1:
      digitalWrite(motor1_pin, LOW);
      digitalWrite(motor2_pin, HIGH);
      break;
    case 0:
      digitalWrite(motor1_pin, LOW);
      digitalWrite(motor2_pin, LOW);
      break;
    case 1:
      digitalWrite(motor1_pin, HIGH);
      digitalWrite(motor2_pin, LOW);
      break;
  }
  analogWrite(speed_pin, Speed);
}

mclass motor = mclass();