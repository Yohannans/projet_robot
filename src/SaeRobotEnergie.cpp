/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   SaeRobotEnergie.cpp                                :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yansquer <yansquer@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2026/01/14 21:25:32 by yansquer          #+#    #+#             */
/*   Updated: 2026/01/14 21:31:14 by yansquer         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#define BASE_SPEED 150 // Vitesse de base du robot
#define COR_COEF 75 // Coefficient de correction de la vitesse en fonction du décalage
#define JACK 11 // Pin de la tirette jack
#define FLOOR_SENSOR 16 // Pin du capteur de sol (photo-diode)
#define PWMG 10  // Signal PWM Gauche vers le pont en H
#define PWMD 9   // Signal PWM Droit vers le pont en H
#define SENSG 8  // Signal SENS Gauche vers le pont en H
#define SENSD 7  // Signal SENS Droit vers le pont en H
#define SEUIL_PIN 2 // Pin du comparateur du capteur milieu
#define COEF_ROT 2.5 // Coefficient de réduction de la vitesse en rotation

#include "Arduino.h" // Utilisation des fonctions Arduino dans Platformio

const uint8_t sensor_port[3] = {A0, A1, A6};

int distance[3]; // Distance en centimètres (Centre = 0, Gauche = 1, Droite = 2)
int speed[2]; // Vitesse des moteurs (Gauche = 0, Droite = 1)
float delta; // Différence relative entre les distances gauche et droite

void motor_speed(int speed[]); // Fonction pour régler la vitesse des moteurs
void refresh_distance(void); // Fonction pour actualiser les distances lues par les capteurs
void orientation_mode(void); // Fonction pour orienter le robot quand il est proche d'un obstacle

void setup() {
  // Initialisation des pins de commande des moteurs
  pinMode(PWMD, OUTPUT);
  pinMode(PWMG, OUTPUT);
  pinMode(SENSD, OUTPUT);
  pinMode(SENSG, OUTPUT);
  // Initialisation des pins des capteurs
  pinMode(FLOOR_SENSOR, INPUT);
  pinMode(SEUIL_PIN, INPUT);
  pinMode(JACK, INPUT);
  while (digitalRead(JACK) == LOW) {  // Attente de la tirette jack
    delay(1);
  }
  // Avancer jusqu'à ce que le robot soit hors de la ligne de départ
  speed[0] = BASE_SPEED; 
  speed[1] = BASE_SPEED;
  motor_speed(speed);
  while (digitalRead(FLOOR_SENSOR) == HIGH) {
    delay(1);
  }
}

void loop() {
  refresh_distance(); // Actualisation des distances
  if (digitalRead(FLOOR_SENSOR) == HIGH) { // Si le robot est sur la ligne blanche
    stop_robot(); // Arrêt du robot
  }
  delta = (float)(distance[1] - distance[2]) / (float)(distance[1] + distance[2]); // Calcul du décalage relatif
  if (digitalRead(SEUIL_PIN)) // Si le capteur central n'est pas proche d'un obstacle
  {
    // Ajustement des vitesses en fonction du décalage
    speed[0] = (BASE_SPEED + delta * COR_COEF); 
    speed[1] = (BASE_SPEED - delta * COR_COEF);
  }
  else
  {
    orientation_mode();
  }
  motor_speed(speed); // Application des vitesses aux moteurs
  delay(1); // Petite pause pour la stabilité
}

void motor_speed(int speed[])
{
  digitalWrite(SENSD, speed[1] < 0 ? HIGH : LOW); // Sens du moteur droit
  digitalWrite(SENSG, speed[0] < 0 ? LOW : HIGH); // Sens du moteur gauche
  analogWrite (PWMD, abs(speed[0])); // Vitesse du moteur droit
  analogWrite (PWMG, abs(speed[1])); // Vitesse du moteur gauche
}

void refresh_distance(void)
{
  static int previous_distance[3] = {0, 0, 0}; // Stockage des distances précédentes pour la moyenne mobile
  for(int i = 0; i < 3; i++)
  {
    float volts = analogRead(sensor_port[i])*0.0048828125;  // Conversion de la lecture analogique en volts
      distance[i] = (13 * pow(volts, -1) + previous_distance[i] * 2) / 3; // Calcul de la distance avec une moyenne mobile
      previous_distance[i] = distance[i]; // Mise à jour de la distance précédente
  }
}

void orientation_mode(void)
{
  bool turn_left = distance[1] > distance[2]; // Détermination du sens de rotation
  // Freinage avant de tourner
  speed[0] = -100; 
  speed[1] = -100;
  motor_speed(speed);
  delay(300);
  // Rotation jusqu'à ce que le capteur central ne soit plus proche d'un obstacle
  while (!digitalRead(SEUIL_PIN))
  {
    refresh_distance();
    if (turn_left)
    {
      speed[0] = -BASE_SPEED / COEF_ROT;
      speed[1] = BASE_SPEED / COEF_ROT;
    }
    else
    {
      speed[0] = BASE_SPEED / COEF_ROT;
      speed[1] = -BASE_SPEED / COEF_ROT;
    }
    motor_speed(speed);
  }
}

void stop_robot(void)
{
  // Freinage
  speed[0] = -100;
  speed[1] = -100;
  motor_speed(speed);
  delay(500);
  // Arrêt complet
  speed[0] = 0;
  speed[1] = 0;
  motor_speed(speed);
  exit(0); // Fin du programme
}

/*
                             __
                            /  \      __
.---.                  _   /   /   _.~  \
\    `.               / \ /   /.-~    __/
 `\    \              |  |   |/    .-~ __
   \    \             |  |   |   .'--~~  \
    \    \            |  |   `  ' _______/
     \    \           |  `        /
 .--. \    \          |    `     /
 \   `.\    \          \        /
  `\   \     \          `\     (
    \   \     \           > ,-.-.
     \   `.    \         /  |  \ \
      \    .    \       /___| O |O\     ,
   .-. \    ;    |    /`    `^-.\.-'`--'/
   \  `;         |   |                 /
    `\  \        |   `.     `--..____,'
      \  `.      |     `._     _.-'^
       \   .     /         `|`|`
     .-.\       /           | |
     \  `\     /            | |
      `\  `   |             | |
        \     |             | |
       .-.    |             | |
       \  `.   \            | |
        `\      \           | |
          \      \          | |
           \_____ :-'~~~~~'-' ;
           /____;``-.         :
          <____(     `.       ;
            \___\     ;     .'
               /``--'~___.-'
              /\___/^/__/
             /   /' /`/'
             \  \   `\ \
              `\ \    \ \
                \ \    \ \
                 \ \    \ \
                  \ \    \ \     ______
                   \ \ ___\ \'~``______)>
              jgs   \ \___ _______ __)>
                _____\ \'~``______)>
              <(_______.._______)>
*/

