#include <Servo.h>

Servo myservoleft;
Servo myservoright;

const float Kp = 3.5;
const float Ki = 0.01;
const float Kd = 0.9;
const int iniMotorPower = 0;

int analogPin = 0;
int val = 0;

float P = 0;
float I = 0;
float D = 0;
int previousError = 0;
int error = 0;

float timestamp;
float dt = 0;
 
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

int current_difference = 0;


const int speed = 6;

void setSpeedLeft(float value){
  // 89 is the value to set in order to stop the servo motors
  myservoleft.write(89 + speed - round(value/2));
}

void setSpeedRight(float value){
  // 89 is the value to set in order to stop the servo motors
  myservoright.write(89 - speed - round(value/2));
}

int l = 0;
int c = 0;
int r = 0;

void setup() {
    Serial.begin(9600);
    myservoleft.attach(8);
    myservoright.attach(7);
    // 89 is the value to set in order to stop the servo motors
    myservoright.write(89);
    myservoleft.write(89);

    // average
    l = analogRead(0);
    c = analogRead(1);
    r = analogRead(2);
}

void loop() {
  val = analogRead(analogPin);
  //Serial.println(val - 70);

  int left = l - analogRead(0);
  int center = c - analogRead(1);
  int right = r - analogRead(2);

  // Complementary filter
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

 /* Serial.print(round(left_media));
  Serial.print(" ");
  Serial.print(round(center_media));
  Serial.print(" ");
  Serial.print(round(right_media));
  Serial.print(" ");*/


  previous_center = center;
  previous_left = left;
  previous_right = right;
  current_difference = round((right_media-center_media) - (left_media-center_media));
  Serial.println(current_difference);
  //Serial.print(" ");
  int value = calculatePID(current_difference , 0);
  //Serial.println(value);
  //Serial.print(" ");

  //Serial.println(value);

  setSpeedLeft(value);
  setSpeedRight(value);
  //myservoright.write(100);
  //myservoleft.write(89);
}

float calculatePID(int currentValue, int target)
{ 
  dt = millis() - timestamp;
  error = target - currentValue;
  P = error;
  I = I + error*(dt/1000);
  D = (error-previousError)/(dt/1000);
  previousError = error;
  timestamp = millis();
  return (Kp*P) + (Ki*I) + (Kd*D);
}

