const int PWM_FOR_PIN = 9;  // Driver du moteur : PWM Forward input 
const int PWM_REV_PIN = 10; // Driver du moteur : PWM Reversed input 
const int STBY_PIN = 8;  // Driver du moteur : Standby input
const int MOTOR_CURRENT = A1; // Driver du moteur : mesure du courant
const int MOTOR_VOTAGE = A0; // Driver du moteur : mesure de la tension
const int TEST_BT = 12; // Bouton de test
const int SPEED_POT = A2; // Potentiomètre réglage vitesse moteur

float current = 0;
float voltage = 0;
int test_bt_state = 0;
int speed_pot_val = 0;

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
  pinMode(TEST_BT, INPUT);

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
}

void loop() {
  test_bt_state = digitalRead(TEST_BT);
  speed_pot_val = map(analogRead(SPEED_POT), 0, 1023, 0, 255);
  if (test_bt_state == HIGH) {
    test_motor();
  }
}

void test_motor() {
  digitalWrite(PWM_REV_PIN, LOW); // Set motor direction to forward
  analogWrite(PWM_FOR_PIN, speed_pot_val);
  digitalWrite(LED_BUILTIN, HIGH);
  mesure();
  delay(2000); // Run motor for 2 seconds
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PWM_FOR_PIN, LOW); // Set motor direction to reverse
  analogWrite(PWM_REV_PIN, speed_pot_val);
  mesure();
  delay(2000); // Run motor for 2 seconds
  stopMotor();
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
