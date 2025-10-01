const int PWM_FOR_PIN = 9;  // Driver du moteur : PWM Forward input 
const int PWM_REV_PIN = 10; // Driver du moteur : PWM Reversed input 
const int STBY_PIN = 8;  // Driver du moteur : Standby input
const int MOTOR_CURRENT = A1; // Driver du moteur : mesure du courant
const int MOTOR_VOTAGE = A0; // Driver du moteur : mesure de la tension
const int TEST_BT = 2; // Bouton de test
const int SPEED_POT = A2; // Potentiomètre réglage vitesse moteur

float current = 0; // Valeur du courant
float voltage = 0; // Valeur de la tension d'alimentation
volatile bool servo_state = 0; // Stocker l'état du servomoteur : 1 en mouvement, 0 pas en mouvement
volatile bool servo_dir = true; // Direction du mouvement du servomoteur : TRUE remontée, FALSE descente
volatile unsigned prev_time_bt = 0; // Pour éviter l'effot bouncing du bouton
int speed_pot_val = 0; // % vitesse moteur via le potentiomètre

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate

  // Configurer les interruptions
  pinMode(TEST_BT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TEST_BT), toogleBt, RISING);

  // Set motor driver pins as outputs
  pinMode(PWM_FOR_PIN, OUTPUT);
  pinMode(PWM_REV_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  // Enable the motor driver
  digitalWrite(STBY_PIN, HIGH);
  stopMotor();

  pinMode(LED_BUILTIN, OUTPUT);

}

void stopMotor() {
  digitalWrite(PWM_REV_PIN, LOW);
  digitalWrite(PWM_FOR_PIN, LOW);
  servo_dir = true;
  servo_state = 0;
}

void toogleBt() {
  if (millis() - prev_time_bt >= 250){
    prev_time_bt = millis();
    if (servo_state == 0) {
      servo_state = 1;
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      stopMotor();
    }
  }
}

void loop() {
  if (servo_state == 1) {
    testMotor();
  }
}

void testMotor() {
  speed_pot_val = map(analogRead(SPEED_POT), 0, 1023, 0, 255);
  if (servo_dir) {
    digitalWrite(PWM_REV_PIN, LOW); // Set motor direction to forward
    analogWrite(PWM_FOR_PIN, speed_pot_val);
  } else {
    digitalWrite(PWM_FOR_PIN, LOW); // Set motor direction to reverse
    analogWrite(PWM_REV_PIN, speed_pot_val);
  }
  mesure();
  delay(2000); // Run motor for 2 seconds
  if (servo_dir == false) {
    toogleBt();
  } else if (servo_state == 1) {
    servo_dir = !(servo_dir);
  }
  
}

void mesure() {
  current = ((analogRead(MOTOR_CURRENT) / 1023.0) * 5.0 - (5.0/2)) / 0.185; // Convert to current
  voltage = (float)(analogRead(MOTOR_VOTAGE) * 5.0 * 5.0)/ 1023.0; // Convert to voltage
  print_mesure();
}

void print_mesure() {
  Serial.print("Courant : ");
  Serial.print(current,3);
  Serial.println(" A");
  Serial.print("Voltage : ");
  Serial.print(voltage,3);
  Serial.println(" V");
}
