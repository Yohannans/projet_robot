#define BASE_SPEED 150
#define COR_COEF 70
#define DISTANCE_SEUIL 0
#define SEUIL_COTE 10
#define JACK 11
#define FLOOR_SENSOR 16

#include "Arduino.h"

const int PWMG = 10; // Signal PWM Gauche vers le pont en H
const int PWMD = 9 ; // Signal PWM Droit vers le pont en H
const int SENSG = 8; // Signal SENS Gauche vers le pont en H
const int SENSD = 7; // Signal SENS Droit vers le pont en H
const int SEUIL_PIN = 2;
const uint8_t sensor_port[3] = {A0, A1, A6};
int speedValue=0; // speed value
String readValue; // Incoming serial data
int distance[3]; // center, left, right
int speed[2]; // left, right
float delta; //Yolo

void motor_speed(int speed[]);
void refresh_distance(void);
void orientation_mode(void);

void setup() {
  pinMode(PWMD, OUTPUT);
  pinMode(PWMG, OUTPUT);
  pinMode(SENSD, OUTPUT);
  pinMode(SENSG, OUTPUT);
  Serial.begin(9600); // open a serial connection
  while (digitalRead(JACK) == LOW) {
    // Wait until the jack is plugged in
    delay(1);
  }
  speed[0] = BASE_SPEED;
  speed[1] = BASE_SPEED;
  motor_speed(speed);
  while (digitalRead(FLOOR_SENSOR) == HIGH) {
    // Wait until the robot is on the ground
    delay(1);
  }
}

void loop() {
  refresh_distance();
  if (digitalRead(FLOOR_SENSOR) == HIGH) {
    // Stop if the robot is lifted
    speed[0] = -100;
    speed[1] = -100;
    motor_speed(speed);
    delay(500);
    // while (digitalRead(FLOOR_SENSOR) == LOW) {
    //   delay(1);
    // }
    speed[0] = 0;
    speed[1] = 0;
    motor_speed(speed);
    while (1) {
      delay(1);
    }
    return;
  }
  delta = (float)(distance[1] - distance[2]) / (float)(distance[1] + distance[2]);
  if (digitalRead(SEUIL_PIN))
  {
    speed[0] = (BASE_SPEED + delta * COR_COEF);
    speed[1] = (BASE_SPEED - delta * COR_COEF);
  }
  else
  {
    orientation_mode();
  }
  motor_speed(speed);
  delay(1);
  // Serial.println(digitalRead(FLOOR_SENSOR));
  // for(int i = 0; i < 3; i++)
  // {
  //   Serial.print("Capteur ");
  //   Serial.print(i);
  //   Serial.print(" : ");
  //   Serial.print(distance[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");
  // Serial.print("Seuil : ");
  // Serial.println(digitalRead(SEUIL_PIN));
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
    static int previous_distance[3] = {0, 0, 0}; // Store previous readings
  for(int i = 0; i < 3; i++)
  {
    float volts = analogRead(sensor_port[i])*0.0048828125;  // value from sensor * (5/1024)
      distance[i] = (13 * pow(volts, -1) + previous_distance[i] * 2) / 3; // Moving average
      previous_distance[i] = distance[i]; // Update previous reading
  }
}

void orientation_mode(void)
{
  bool turn_left = distance[1] > distance[2];
  speed[0] = -100;
  speed[1] = -100;
  motor_speed(speed);
  delay(300);
  while (!digitalRead(SEUIL_PIN))
  {
    refresh_distance();
    if (turn_left)
    {
      speed[0] = -BASE_SPEED/2.5;
      speed[1] = BASE_SPEED/2.5;
    }
    else
    {
      speed[0] = BASE_SPEED/2.5;
      speed[1] = -BASE_SPEED/2.5;
    }
    motor_speed(speed);
  }
}