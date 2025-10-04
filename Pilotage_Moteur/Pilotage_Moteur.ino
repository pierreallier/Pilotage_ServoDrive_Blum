// Basé sur le travail de 3sigma : https://blog.3sigma.fr/arduino/tutoriel-arduino-asservissement-en-vitesse-dun-moteur-a-courant-continu/

#include <FlexiTimer2.h>
#include <digitalWriteFast.h> 

// Codeur incrémental
#define CODEUR_A_PIN 18
#define CODEUR_B_PIN 19
volatile long ticksCodeur = 0;

// Autres capteurs
#define CODEUR_PORTE A4

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

#define TEST_BT 2 // Bouton de mise en fonctionnement / arrêt
volatile bool motor_active = LOW;
volatile unsigned prev_time_bt = 0; // Pour éviter l'effot bouncing du bouton

// Initialisations
void setup(void) {

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
}

void toogleBt() {
  if (millis() - prev_time_bt >= 250){
    prev_time_bt = millis();
    if (motor_active  == LOW) {
      digitalWrite(STBY_PIN, HIGH);
      motor_active = HIGH;
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      motor_active = LOW;
      digitalWrite(STBY_PIN, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void isrt(){

  int codeurDeltaPos;
  int commande;
  
  // Nombre de ticks codeur depuis la dernière fois
  codeurDeltaPos = ticksCodeur;
  ticksCodeur = 0;

  // Calcul de la vitesse de rotation
  motor_speed = ((2.*3.141592*((double)codeurDeltaPos))/128.)/dt;  // en rad/s
  motor_angle += (double)(codeurDeltaPos)*360/128.; // en °

  // Envoi de la commande au moteur
  commande = (int)((analogRead(SPEED_POT)-500)/2.);
  if (abs(commande) < 100) {
    commande = 0;
  }
  CommandeMoteur(commande);

  motor_command = commande/255.*(float)(analogRead(MOTOR_VOLTAGE) * 25)/ 1023.0; // en V
  door_angle = map(analogRead(CODEUR_PORTE), 0, 1023, 0, 360); // en °
  motor_current = ((analogRead(MOTOR_CURRENT) / 1023.0) * 5.0 - (5.0/2)) / 0.185; // en A
  driver_current = ((analogRead(DRIVER_CURRENT) / 1023.0) * 5.0 ) / 0.500; // en A
  temps += dt; // en s
}

void ecritureData(void) {

  // Ecriture des données en sortie tous les TSDATA millisecondes
  tempsCourant = millis();
  if (tempsCourant-tempsDernierEnvoi > TSDATA) {
    Serial.print(temps);

    Serial.print(",");
    Serial.print("Commande moteur(V):");
    Serial.print(motor_command); // Tension moyenne moteur
    Serial.print(",");
    Serial.print("Courant moteur(A):");
    Serial.print(motor_current); // courant moteur
    Serial.print(",");
    Serial.print("Courant driver(A):");
    Serial.print(driver_current); // Courant driver = courant moteur
    Serial.print(",");
    Serial.print("Vitesse moteur(rad/s):");
    Serial.print(motor_speed); // Vitesse rotation moteur
    Serial.print(",");
    //Serial.print("Angle moteur(°):");
    //Serial.print(motor_angle); // Angle arbre moteur
    //Serial.print(",");
    Serial.print("Angle porte(°):");
    Serial.print(door_angle); // Angle de la porte
    
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

  // Commande PWM
	if (tension_int>0) {
    digitalWrite(PWM_REV_PIN, LOW); // Set motor direction to forward
    analogWrite(PWM_FOR_PIN, tension_int);
	}
	else if (tension_int<0) {
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