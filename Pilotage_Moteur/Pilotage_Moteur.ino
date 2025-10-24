// Basé sur le travail de 3sigma : https://blog.3sigma.fr/arduino/tutoriel-arduino-asservissement-en-vitesse-dun-moteur-a-courant-continu/

#include <FlexiTimer2.h>
#include <digitalWriteFast.h> 

// Codeur incrémental
#define CODEUR_A_PIN 18
#define CODEUR_B_PIN 19
volatile long ticksCodeur = 0;

// Codeur porte
#define CODEUR_PORTE A4
bool is_limite_haute = LOW;
bool is_limite_basse = LOW;
const int limite_haute_val = 180;
const int limite_basse_val = -135;
const int codeur_porte_decalage = 12;

// Moteur CC
#define PWM_REV_PIN  4
#define PWM_FOR_PIN  5
#define STBY_PIN 8  // Driver du moteur : Standby input
#define MOTOR_VOLTAGE A0 // Driver du moteur : mesure de la tension
#define MOTOR_CURRENT A1
#define DRIVER_CURRENT A3
#define SPEED_POT A2 // Potentiomètre réglage vitesse moteur

// Cadence d'envoi des données en ms
#define TSDATA 100
unsigned long tempsDernierEnvoi = 0;
unsigned long tempsCourant = 0;

// Cadence d'échantillonnage en ms
#define CADENCE_MS 10
volatile double dt = CADENCE_MS/1000.;
volatile double temps = -CADENCE_MS/1000.;

volatile double motor_speed = 0.;
volatile double motor_command = 0.;
volatile double motor_current = 0.;
volatile double driver_current = 0.;
volatile double door_angle = 0.;
volatile double motor_angle = 0.;
volatile bool motor_dir=LOW; // LOW=ouverture, HIGH=fermeture

#define TEST_BT 2 // Bouton de mise en fonctionnement / arrêt
volatile bool motor_active = LOW;
volatile bool stop_motor = LOW;
volatile unsigned prev_time_bt = 0; // Pour éviter l'effot bouncing du bouton

// Moyenne courant
volatile bool is_current_limit_reach = LOW;
const double limite_current = 2;
const unsigned int current_moy_nb = 5;
volatile unsigned int current_index = 0;
volatile double current_moy_val = 0.;
volatile double current_tab[current_moy_nb];

// Asservissement
bool is_bf = LOW;
volatile double consigne = 0.;
volatile double commande = 0.;
volatile double Kp = 10.;
volatile double Ki = 5.;
volatile double Kd = 0.;
volatile double P_x = 0.;
volatile double I_x = 0.;
volatile double D_x = 0.;
volatile double ecart = 0.;

// Initialisations
void setup(void) {
  // Initialisation du tableau du courant
  for (int i=0; i < current_moy_nb; i++) {
    current_tab[i] = 0;
  }

  // Codeur incrémental
  pinMode(CODEUR_A_PIN, INPUT);      // entrée digitale pin A codeur
  pinMode(CODEUR_B_PIN, INPUT);      // entrée digitale pin B codeur
  digitalWrite(CODEUR_A_PIN, HIGH);  // activation de la résistance de pullup
  digitalWrite(CODEUR_B_PIN, HIGH);  // activation de la résistance de pullup
  attachInterrupt(digitalPinToInterrupt(CODEUR_A_PIN), GestionInterruptionCodeurPinA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CODEUR_B_PIN), GestionInterruptionCodeurPinB, CHANGE);

  // Bouton de mise en fonctionnement
  pinMode(TEST_BT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TEST_BT), toogleBt, RISING);
  pinMode(LED_BUILTIN, OUTPUT);

  // Moteur CC
  pinMode(PWM_REV_PIN, OUTPUT);
  pinMode(PWM_FOR_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW); // Enable the motor driver
  digitalWrite(PWM_REV_PIN, LOW);
  digitalWrite(PWM_FOR_PIN, LOW);

  // Liaison série
  Serial.begin(9600);
  Serial.flush();

  // Compteur d'impulsions de l'encodeur
  ticksCodeur = 0;

  // La routine isrt est exécutée à cadence fixe
  FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // résolution timer = 1 ms
  FlexiTimer2::start();

}

// Boucle principale
void loop() {
  // Ecriture des données sur la liaison série
  if (motor_active) {
    ecritureData();
  }
  if (stop_motor) {
    motor_active = LOW;
    if (motor_dir) {
      CommandeMoteur(-255);
    }
    else {
      CommandeMoteur(255);
    }
    delay(30);
    digitalWrite(STBY_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    stop_motor = LOW;
  }
}

void toogleBt() {
  if (millis() - prev_time_bt >= 250){
    prev_time_bt = millis();
    if (motor_active  == LOW) {
      digitalWrite(STBY_PIN, HIGH);
      motor_active = HIGH;
      is_current_limit_reach = LOW;
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      stopMotor();
    }
  }
}

void stopMotor() {
  stop_motor = HIGH;
  //motor_active = LOW;
  //digitalWrite(STBY_PIN, LOW);
  //digitalWrite(LED_BUILTIN, LOW);
}

void isrt(){

  int codeurDeltaPos;
  
  // Nombre de ticks codeur depuis la dernière fois
  codeurDeltaPos = ticksCodeur;
  ticksCodeur = 0;

  // Calcul de la vitesse de rotation
  motor_speed = ((3.141592*((double)codeurDeltaPos))/128.)/dt;  // en rad/s
  motor_angle += (double)(codeurDeltaPos)*180/128.; // en °

  // Vérification des limites angulaire de la porte
  door_angle = getAngleDoorCorr(analogRead(CODEUR_PORTE));
  if (door_angle >= limite_haute_val) {
    is_limite_haute = HIGH;
  } else {
    is_limite_haute = LOW;
  }
  if (door_angle <= limite_basse_val) {
    is_limite_basse = HIGH;
  } else {
    is_limite_basse = LOW;
  }
  
  if (is_bf) {
    // Commande en boucle fermée : le potentiomètre définit l'angle souhaité
    consigne = (int)((analogRead(SPEED_POT)-500)*0.1);
    D_x = -ecart*Kd;
    ecart = consigne*2.4 - door_angle;
    D_x += ecart*Kd;
    P_x = Kp*ecart; // Terme proportionnel
    commande = P_x + I_x + D_x; // Calcul de la commande
    I_x = I_x + Ki*dt*ecart; // Terme intégrale (sera utilisé lors du pas déchantillonnage suivant)
    commande = commande * -2.;
  }
  else {
    // Commande en boucle ouverte : le potentiomètre définit la vitesse souhaité
    consigne = (int)((analogRead(SPEED_POT)-500)/2.);
    commande = consigne;
    if (abs(commande) < 100) {
      commande = 0;
    }
  }
  if (motor_active) {
    CommandeMoteur((int)(commande));
  }

  motor_current = ((analogRead(MOTOR_CURRENT) / 1023.0) * 5.0 - (5.0/2)) / 0.185; // en A
  driver_current = ((analogRead(DRIVER_CURRENT) / 1023.0) * 5.0 ) / 0.500; // en A
  door_angle = door_angle / 2.4; // angle réel de la porte en °
  temps += dt; // en s

  if (addCurrent(driver_current) >= limite_current) {
    is_current_limit_reach = HIGH;
    stopMotor();
  }
}

int getAngleDoorCorr(int angle){
  door_angle = map(angle,0,655,0,360.0)-codeur_porte_decalage;
  if (door_angle > 210) {
    door_angle -= 360; 
  }
  return door_angle;
}

double addCurrent(double current) {
  // Fonction qui calcule une moyenne glissante
  current_moy_val -= current_tab[current_index];
  current_moy_val += current;
  current_tab[current_index] = current;
  current_index = (current_index+1) % current_moy_nb;
  return (float) current_moy_val / current_moy_nb;
}

void ecritureData(void) {

  // Ecriture des données en sortie tous les TSDATA millisecondes
  tempsCourant = millis();
  if (tempsCourant-tempsDernierEnvoi > TSDATA) {
    Serial.print(temps);

    Serial.print(",Consigne(° si BF):");
    Serial.print(consigne); // consigne angulaire si BF, commande hacheur si BO
    Serial.print(",Commande hacheur:");
    Serial.print(commande); // commande hacheur
    Serial.print(",Tension moteur(V):");
    Serial.print(motor_command); // Tension moyenne moteur
    Serial.print(",Courant moteur(A):");
    Serial.print(motor_current); // courant moteur
    Serial.print(",Courant driver(A):");
    Serial.print(driver_current); // Courant driver = courant moteur
    Serial.print(",Vitesse moteur(rad/s):");
    Serial.print(motor_speed); // Vitesse rotation moteur
    Serial.print(",Angle moteur(°):");
    Serial.print(motor_angle); // Angle arbre moteur
    Serial.print(",Angle porte(°):");
    Serial.print(door_angle); // Angle de la porte
    Serial.print(", bool(moteur actif, limite haute, limite basse, courant): ");
    Serial.print(motor_active);
    Serial.print(is_limite_haute);
    Serial.print(is_limite_basse);
    Serial.print(is_current_limit_reach);
    Serial.print("\r");
    Serial.print("\n");

    tempsDernierEnvoi = tempsCourant;
  }
}

void CommandeMoteur(int tension_int)
{
	// Saturation par sécurité
	if (tension_int>255) {
		tension_int = 255;
	}
	if (tension_int<-255) {
		tension_int = -255;
	}
  motor_command = tension_int/255.*(float)(analogRead(MOTOR_VOLTAGE) * 25)/ 1023.0; // en V

  // Commande PWM
	if (tension_int>0 & ~is_limite_basse) {
    motor_dir = HIGH;
    digitalWrite(PWM_REV_PIN, LOW); // Set motor direction to forward
    analogWrite(PWM_FOR_PIN, tension_int);
	}
	else if (tension_int<0 & ~is_limite_haute) {
    motor_dir = LOW;
		digitalWrite(PWM_FOR_PIN, LOW); // Set motor direction to reverse
    analogWrite(PWM_REV_PIN, -tension_int);
	}
  else {
    digitalWrite(PWM_REV_PIN, LOW);
    digitalWrite(PWM_FOR_PIN, LOW);
  }
}

// Routine de service d'interruption attachée à la voie A du codeur incrémental
void GestionInterruptionCodeurPinA()
{
  if (digitalReadFast2(CODEUR_A_PIN) == digitalReadFast2(CODEUR_B_PIN)) {
    ticksCodeur--;
  }
  else {
    ticksCodeur++;
  }
}

// Routine de service d'interruption attachée à la voie B du codeur incrémental
void GestionInterruptionCodeurPinB()
{
  if (digitalReadFast2(CODEUR_A_PIN) == digitalReadFast2(CODEUR_B_PIN)) {
    ticksCodeur++;
  }
  else {
    ticksCodeur--;
  }
}