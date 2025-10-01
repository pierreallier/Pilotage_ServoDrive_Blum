const int PWM_FOR_PIN = 9;  // Driver du moteur : PWM Forward input 
const int PWM_REV_PIN = 10; // Driver du moteur : PWM Reversed input 
const int STBY_PIN = 8;  // Driver du moteur : Standby input
const int MOTOR_CURRENT = A1; // Driver du moteur : mesure du courant
const int MOTOR_VOTAGE = A0; // Driver du moteur : mesure de la tension

float current = 0;
float voltage = 0;

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate

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
  if (false) {
    digitalWrite(PWM_REV_PIN, LOW); // Set motor direction to forward
    analogWrite(PWM_FOR_PIN, 128); // Set motor speed to 50% (128 out of 255)
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000); // Run motor for 2 seconds
    current = ((analogRead(MOTOR_CURRENT) / 1023.0) * 5.0 - (5.0/2)) / 0.185; // Convert to current
    voltage = (float)(analogRead(MOTOR_VOTAGE) * 5.0 * 5.0)/ 1023.0; // Convert to voltage
    Serial.print("Courant : ");
    Serial.print(current);
    Serial.println("A");
    Serial.print("Voltage : ");
    Serial.print(voltage);
    Serial.println("V");
    delay(1000); // Run motor for 2 seconds
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(PWM_FOR_PIN, LOW); // Set motor direction to reverse
    analogWrite(PWM_REV_PIN, 192);// Set motor speed to 75% (192 out of 255)
    delay(1000); // Run motor for 2 seconds
    current = ((analogRead(MOTOR_CURRENT) / 1023.0) * 5.0 - (5.0/2)) / 0.185; // Convert to current
    voltage = (float)(analogRead(MOTOR_VOTAGE) * 5.0 * 5.0)/ 1023.0; // Convert to voltage
    Serial.print("Courant : ");
    Serial.print(current,3);
    Serial.println(" A");
    Serial.print("Voltage : ");
    Serial.print(voltage);
    Serial.println("V");
    delay(1000); // Run motor for 2 seconds
  }
}
