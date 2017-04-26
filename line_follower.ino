#include <Servo.h>

Servo myservoleft;
Servo myservoright;

const int Kp = 6;
const int Ki = 5;
const int Kd = 1;
const int iniMotorPower = 0;

int analogPin = 0;    
int val = 0;

int P = 0;
int I = 0;
int D = 0;
int PIDvalue = 0;
int previousError = 0;
int error = 0;

unsigned long timestamp;
unsigned long dt = 0;

int previous_center = 0;
float alpha = 0.5;
int current_center = 0;
int myInts[6];



void setup() {
    Serial.begin(9600);
    myservoleft.attach(7);
    myservoright.attach(8);
    myservoright.write(98);
    myservoleft.write(80);
}

void loop() {
  val = analogRead(analogPin);
  //Serial.println(val - 70);
  
  int left = analogRead(0);
  int center = analogRead(1);
  int right = analogRead(2);
  
  //Serial.print(left);
  //Serial.print("   ");
  
  current_center = round((1-alpha)*previous_center + alpha*center);
  Serial.println(center - current_center);
  previous_center = center;
  //Serial.print("   "); 
  //Serial.println("");

  
  /*Serial.print(analogRead(2) - 70);
  Serial.print(" ");
  Serial.print(analogRead(1) - 70);
  Serial.print(" ");
  Serial.print(analogRead(0) - 70);
  Serial.println("");*/

  int value = calculatePID( (right-center) - (left-center) , 0);
  //Serial.println(value);
  
  if (Serial.available()) { 
  // Returns true if there is serial input.
    myservoright.write(89);
    myservoleft.write(89);
  } else {
    myservoright.write(98 + round(value/2) );
    myservoleft.write(80 + round(value/2) );
  }
}

int calculatePID(int currentValue, int target)
{
  dt = millis() - timestamp;
  error = target - currentValue;
  P = error;
  I = I + error*(dt/1000);
  D = (error-previousError)/(dt/1000);
  previousError = error;
  timestamp = millis();
  return PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
}
