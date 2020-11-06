#include <EnableInterrupt.h>          //insufficient number of standard Interrupt pins on the Arduino Uno
#include <digitalWriteFast.h>         

#define encoderA 2
#define encoderB 3
#define motorEncoder 4
#define motor1 5
#define motor2 6
#define leftLimit 7
#define rightLimit 8
#define motorSpeed 9


volatile int pulseCount = 0;
int pulsesPerRev = 1024;     //This variable will is unique to the type of encoder
int motorDir = 0;   // 1 = CW, 0 = stationary,  -1 = CCW
int encoderDir = 0;  

int currCount = 0;
int prevCount;

void setup() {
  Serial.begin(9600);
  pinMode(encoderA, INPUT);
  pinModeFast(encoderB, INPUT);
  pinMode(motorEncoder, INPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(leftLimit, INPUT);
  pinMode(rightLimit, INPUT);
  pinMode(motorSpeed, OUTPUT);
  enableInterrupt(encoderA, pulse, RISING);
  //enableInterrupt(leftLimit, limit, RISING);
  //enableInterrupt(rightLimit, limit, RISING);
  recalibratePendulum();   // wait until cart reaches end of track 
}

void loop() {
  //Serial.print(pendulumAngle());
  //Serial.print('\t');
  //Serial.println();  
  checkLimitSwitches();         // do not need to be actively monitored via interrupts because of slow cart speed relative to loop() cycle speed
  //driveMotor(0, 255);
  delay(50);
}

void driveMotor(int Direction, int Speed){
  switch (Direction){
    case -1:
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, HIGH);
      break;
    case 0:
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, LOW);
      break;
    case 1:
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, LOW);
      break;
  }
  analogWrite(motorSpeed, Speed);
}

bool checkLimitSwitches(){
  if((digitalRead(leftLimit) == 1) || (digitalRead(rightLimit) == 1)){
    driveMotor(0, 0);
    recalibratePendulum();
    //move to centre of track, wait until pendulum is manually rotated to vertical
    return true;
  } 
  else{
    return false;
  }
  /*
  Serial.print(digitalRead(leftLimit));
  Serial.print('\t');
  Serial.print(digitalRead(rightLimit));
  Serial.print('\t');
  Serial.println(digitalRead(motorEncoder));
  */
}

void recalibratePendulum(){
  //moves cart to rightmost limit switch and waits for the pendulum to settle
  while(digitalRead(rightLimit) == 0){
    driveMotor(-1, 255);
    delay(1);
  }
  driveMotor(0, 0);
  do{
    prevCount = currCount;  
    delay(250);                   
    currCount = pulseCount;
  }while(prevCount != currCount);     // wait until the pendulum is stationary
  pulseCount = pulsesPerRev/2;        // calibrate s.t. theta = 0, pi when the pendulum is vertical upwards and downwards, respectively. 
}

float pendulumAngle(){
  // converts encoder data to angle, theta = [-pi, pi]
  // assuming function is sampled frequently (it is) pulseCount will never suffer integer overflow
  if(abs(pulseCount) > pulsesPerRev/2){
    int polarity = pulseCount/abs(pulseCount); 
    volatile int divisor = pulseCount/pulsesPerRev;   // will always = 0 (assuming frequent sampling), included for rigor
    volatile int normCount = (pulseCount-divisor*pulsesPerRev)-(pulsesPerRev*polarity);
    pulseCount = normCount; 
  }
  float angle = (pulseCount/1024.)*2*PI;
  return angle;
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
  pulseCount += encoderDir;
}
