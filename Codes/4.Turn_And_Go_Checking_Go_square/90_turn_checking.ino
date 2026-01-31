// --- Motor Pins ---
const int AIN1 = 4; const int AIN2 = 7; const int PWMA = 5;
const int BIN1 = 9; const int BIN2 = 8; const int PWMB = 6;
const int ENCL_A = 2;
const int buzzer = 10;

// --- Calibrated Constants ---
const int TICKS_PER_CELL = 616;
const int TICKS_FOR_90 = 285; 
volatile long leftTicks = 0;

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(ENCL_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCL_A), countTicks, RISING);
  delay(5000); // 5-second safety delay
}

void countTicks() { leftTicks++; }

void brake() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  delay(100);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

void turnRight90() {
  leftTicks = 0;
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); // Left Forward
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); // Right Backward
  while (leftTicks < TICKS_FOR_90) {
    analogWrite(PWMA, 80); analogWrite(PWMB, 80);
  }
  brake();
}

void turnLeft90() {
  leftTicks = 0;
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); // Left Backward
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  // Right Forward
  while (leftTicks < TICKS_FOR_90) {
    analogWrite(PWMA, 80); analogWrite(PWMB, 80);
  }
  brake();
}

void moveForward18cm() {
  leftTicks = 0;
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  while (leftTicks < TICKS_PER_CELL) {
    int speed = (leftTicks > TICKS_PER_CELL - 150) ? 60 : 120;
    analogWrite(PWMA, speed); analogWrite(PWMB, speed);
  }
  brake();
}

void loop() {
  // TEST: Move in a square
  for(int i = 0; i < 4; i++) {
    moveForward18cm();
    delay(500);
    turnRight90();
    delay(500);
  }
  while(1); // Stop after square
}
