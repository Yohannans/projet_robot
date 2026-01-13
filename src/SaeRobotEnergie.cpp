#define BASE_SPEED 50
#define HIGH_SPEED 100
#define REVERSE_SPEED -100
#define COR_COEF 45
#include "pins.hpp"
#include <Arduino.h>

// Tableaux pour stocker les distances mesurées et les vitesses des moteurs
int distance[3]; // [0]=centre, [1]=gauche, [2]=droite (en cm)
int speed[2];    // [0]=gauche, [1]=droite

// Machine à état: 5 états distincts pour le robot
// WAIT: en attente (jack non enfoncé)
// FORWARD: avance droit
// TURN_RIGHT: pivote à droite (obstacle détecté à gauche)
// TURN_LEFT: pivote à gauche (obstacle détecté à droite)
// BACKUP: recule (trop proche d'un obstacle)
enum State { WAIT=0, FORWARD=1, TURN_RIGHT=2, TURN_LEFT=3, BACKUP=4 };
State state = WAIT;

// Variables pour contrôler les transitions d'état
unsigned long lastTransitionMs = 0;
const unsigned long MIN_STATE_DWELL_MS = 150; // Délai minimum pour éviter les basculements rapides

// Seuils de distance (hystérésis) pour les transitions entre états
const int TURN_ENTER_CM = 30;      // Distance minimale avant d'entrer en pivot
const int HIGH_TURN_ENTER_CM = 15; // Distance minimale sur les côtés pour pivot rapide
const int TURN_EXIT_CM  = 45;      // Distance pour sortir du pivot et revenir en avant
const int BACKUP_ENTER_CM = 10;    // Distance minimale avant de reculer
const int BACKUP_EXIT_CM  = 20;    // Distance pour sortir du recul et revenir en avant

void	motor_speed(int speed[]);
void	refresh_distance(void);

void	setup(void)
{
	pinMode(PWMD, OUTPUT);
	pinMode(PWMG, OUTPUT);
	pinMode(SENSD, OUTPUT);
	pinMode(SENSG, OUTPUT);
	pinMode(JACK, INPUT);
	Serial.begin(9600); // open a serial connection
}

// Fonction de transition: met à jour l'état en fonction des capteurs
void updateState() {
  switch (state) {
    case WAIT:
      // En attente: démarrer si le jack est enfoncé
      if (digitalRead(JACK) == HIGH) { 
        state = FORWARD; 
        lastTransitionMs = millis(); 
      }
      break;
      
    case FORWARD:
      // Avance: passer en pivot si un obstacle est détecté (centre ou côtés)
      if (distance[0] < TURN_ENTER_CM || distance[1] < HIGH_TURN_ENTER_CM || distance[2] < HIGH_TURN_ENTER_CM) {
        // Tourner vers le côté le moins encombré
        state = (distance[1] < distance[2]) ? TURN_RIGHT : TURN_LEFT;
        lastTransitionMs = millis();
      }
      break;
      
    case TURN_RIGHT:
    case TURN_LEFT:
      // Pivot: reculer si trop proche, sinon revenir en avant si dégagé
      if (distance[0] < BACKUP_ENTER_CM) { 
        state = BACKUP; 
        lastTransitionMs = millis(); 
      }
      else if (distance[0] >= TURN_EXIT_CM) { 
        state = FORWARD; 
        lastTransitionMs = millis(); 
      }
      break;
      
    case BACKUP:
      // Recul: revenir en avant une fois dégagé
      if (distance[0] >= BACKUP_EXIT_CM) { 
        state = FORWARD; 
        lastTransitionMs = millis(); 
      }
      break;
  }
}

// Fonction d'action: applique les vitesses moteurs selon l'état actuel
void applyState() {
  switch (state) {
    case WAIT:
      // Arrêt complet
      speed[0] = 0;
      speed[1] = 0;
      break;
      
    case FORWARD:
      // Avance droit avec vitesse maximale
      speed[0] = HIGH_SPEED;  // moteur gauche
      speed[1] = HIGH_SPEED;  // moteur droite
      break;
      
    case TURN_RIGHT:
      // Pivote à droite: moteur gauche actif, moteur droite arrêté
      speed[0] = HIGH_SPEED;  // moteur gauche
      speed[1] = 0;           // moteur droite arrêté
      break;
      
    case TURN_LEFT:
      // Pivote à gauche: moteur gauche arrêté, moteur droite actif
      speed[0] = 0;           // moteur gauche arrêté
      speed[1] = HIGH_SPEED;  // moteur droite
      break;
      
    case BACKUP:
      // Recule droit: les deux moteurs à marche arrière
      speed[0] = REVERSE_SPEED; // moteur gauche recule
      speed[1] = REVERSE_SPEED; // moteur droite recule
      break;
  }
}

// Boucle principale du programme
void loop(void)
{
  // 1. Lire les distances des capteurs ultrason
  refresh_distance(); // met à jour le tableau distance[]

  // 2. Mettre à jour la machine à état (seulement après le délai minimum)
  // Ceci évite les basculements rapides dus au bruit des capteurs
  if (millis() - lastTransitionMs >= MIN_STATE_DWELL_MS) {
    updateState();
  }

  // 3. Appliquer les vitesses moteurs correspondant à l'état actuel
  applyState();
  
  // 4. Afficher l'état actuel pour le debug
  Serial.print("State: "); Serial.println((int)state);
  
  // 5. Envoyer les vitesses aux moteurs
  motor_speed(speed);
}

// Fonction de contrôle des moteurs: envoie les signaux PWM et direction aux drivers
void motor_speed(int speed[]) {
  int left  = speed[0];  // Moteur gauche
  int right = speed[1];  // Moteur droite

  // Définir la direction de rotation (avant/arrière) pour chaque moteur
  // Les pins SENSG et SENSD contrôlent la direction (HIGH=avant, LOW=arrière)
  digitalWrite(SENSG, left  < 0 ? LOW : HIGH);
  digitalWrite(SENSD, right < 0 ? HIGH : LOW);

  // Convertir la vitesse signée en PWM positif (0-255)
  int pwmLeft  = constrain(abs(left), 0, 255);
  int pwmRight = constrain(abs(right) - 13, 0, 255); // -13 est une compensation pour équilibrer le moteur droit

  // Envoyer les signaux PWM aux drivers moteurs
  analogWrite(PWMG, pwmLeft);  // Moteur gauche
  analogWrite(PWMD, pwmRight); // Moteur droite
}

// Fonction de lecture des capteurs: lit les 3 capteurs ultrason et filtre les mesures
void refresh_distance(void) {
  static float filt[3] = {0,0,0}; // Tableau de filtrage pour lisser les mesures
  const float alpha = 0.4f; // Coefficient de filtrage (plus petit = plus lisse, mais plus lent)

  // Lire les 3 capteurs (centre, gauche, droite)
  for (int i = 0; i < 3; i++) {
    // 1. Convertir la valeur analogique (0-1023) en tension (0-5V)
    float volts = analogRead(sensor_port[i]) * 0.0048828125f; // 5/1024 = 0.00488...
    
    // 2. Convertir la tension en distance en cm (selon la caractéristique du capteur)
    // Formule: distance = K / tension, où K=13 pour ce type de capteur
    float cm = 13.0f * powf(volts, -1.0f);
    
    // 3. Limiter la distance entre 5 et 200 cm (limites de fiabilité du capteur)
    cm = constrain(cm, 5.0f, 200.0f);
    
    // 4. Appliquer un filtre passe-bas pour lisser les bruits de mesure
    // Formule: filt[i] = alpha * nouvelle_mesure + (1-alpha) * ancienne_mesure
    filt[i] = alpha * cm + (1.0f - alpha) * filt[i];
    
    // 5. Convertir en entier et stocker dans le tableau de distances
    distance[i] = (int)filt[i];
  }
}
