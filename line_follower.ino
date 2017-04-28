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

float alpha = 0.5;

int current_center = 0;
int current_left = 0;
int current_right = 0;

int previous_center;
int previous_left;
int previous_right;

float center_values[10];
float right_values[10];
float left_values[10];

int i = 0; 
int temp = 0;

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
  
 

  center_values[i] = (1-alpha)*previous_center + alpha*center;
  left_values[i] = (1-alpha)*previous_left + alpha*left;
  right_values[i] = (1-alpha)*previous_right + alpha*right;

  float center_media = 0;
  float left_media = 0;
  float right_media = 0;

  if (temp < 10) {
    temp++;
  }
  
  for(int z = 0; z < temp - 1; z++) {
    center_media += center_values[z];
    left_media += left_values[z];
    right_media += right_values[z];
  }

  center_media = center_media/(temp-1);
  left_media = left_media/(temp-1);
  right_media = right_media/(temp-1);

  i = (i + 1)%10;

  /*Serial.print(round(center_media));
  Serial.print(" ");
  Serial.println(center);
  /*Serial.print(" ");
  Serial.print(round(left_media));
  Serial.print(" ");
  Serial.print(left);
  Serial.print(" ");
  Serial.print(round(right_media));
  Serial.print(" ");
  Serial.println(right);*/
  
  previous_center = center;
  previous_left = left;
  previous_right = right;

  int value = calculatePID( (right-center) - (left-center) , 0);
  Serial.println(value);
  
  if (Serial.available()) { 
  // Returns true if there is serial input.
    myservoright.write(89);
    myservoleft.write(89);
  } else {
    myservoright.write(128 + round((value/2)*1.5) );
    myservoleft.write(50 + round((value/2)*1.5) );
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
