#include<Arduino.h>
#define PWMG 10 // Signal PWM Gauche vers le pont en H
#define PWMD 9 // Signal PWM Droit vers le pont en H
#define SENSG 8 // Signal SENS Gauche vers le pont en H
#define SENSD 7 // Signal SENS Droit vers le pont en H
#define CAPT_SOL1 17
#define CAPT_SOL2 16
#define JACK 12

const uint8_t sensor_port[3] = {A0, A1, A6};