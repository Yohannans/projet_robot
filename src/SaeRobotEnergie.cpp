#define BASE_SPEED 100
#include "Arduino.h"

const int PWMG = 10; // Signal PWM Gauche vers le pont en H
const int PWMD = 9 ; // Signal PWM Droit vers le pont en H
const int SENSG = 8; // Signal SENS Gauche vers le pont en H
const int SENSD = 7; // Signal SENS Droit vers le pont en H
const uint8_t sensor_port[3] = {A0, A1, A6};
int speedValue=0; // speed value
String readValue; // Incoming serial data
int distance[3]; // center, left, right
int speed[2]; // left, right
float delta;

void motor_speed(int speed[]);
void refresh_distance(void);

void setup() {
  pinMode(PWMD, OUTPUT);
  pinMode(PWMG, OUTPUT);
  pinMode(SENSD, OUTPUT);
  pinMode(SENSG, OUTPUT);
  Serial.begin(9600); // open a serial connection
}

void loop() {
  refresh_distance();
  delta = (float)(distance[1] - distance[2]) / (float)(distance[1] + distance[2]);
  if (distance[0] > 50)
  {
    speed[0] = (BASE_SPEED - delta);
    speed[1] = (BASE_SPEED + delta);
  }
  else
  {
    speed[0] = 0;
    speed[1] = 0;
  }
  motor_speed(speed);
  // for(int i = 0; i < 3; i++)
  // {
  //   Serial.print("Capteur ");
  //   Serial.print(i);
  //   Serial.print(" : ");
  //   Serial.print(distance[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");
  // delay(1000);

}

void motor_speed(int speed[])
{
  digitalWrite(SENSD, speed[1] < 0 ? HIGH : LOW);
  digitalWrite(SENSG, speed[0] < 0 ? LOW : HIGH);
  analogWrite (PWMD, abs(speed[0]));
  analogWrite (PWMG, abs(speed[1]));
}

void refresh_distance(void)
{
  for(int i = 0; i < 3; i++)
  {
    float volts = analogRead(sensor_port[i])*0.0048828125;  // value from sensor * (5/1024)
    distance[i] = 13 * pow(volts, -1);
  }
}
