// --- Updated Pin Definitions ---
const int AIN1 = 4;   // Left Motor Direction 1
const int AIN2 = 7;   // Left Motor Direction 2
const int PWMA = 5;   // Left Motor Speed (PWM)

const int BIN1 = 9;   // Right Motor Direction 1 (Updated)
const int BIN2 = 8;   // Right Motor Direction 2
const int PWMB = 6;   // Right Motor Speed (PWM) (Updated)

const int STBY = 5;   // Reminder: Connect TB6612 STBY to Arduino 5V pin!

void setup() {
  Serial.begin(9600);
  
  // Set all motor pins as OUTPUT
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.println("--- Motor Direction Test Starting ---");
  Serial.println("Ensure wheels are off the ground!");
}

void loop() {
  // 1. TEST LEFT MOTOR FORWARD
  Serial.println("Left Motor: Forward");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 200); // 100/255 speed
  delay(2000);
  
  // Stop
  analogWrite(PWMA, 0);
  delay(1000);

  // 2. TEST RIGHT MOTOR FORWARD
  Serial.println("Right Motor: Forward");
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 200);
  delay(2000);

  // Stop
  analogWrite(PWMB, 0);
  delay(1000); // Wait 3 seconds before repeating
}