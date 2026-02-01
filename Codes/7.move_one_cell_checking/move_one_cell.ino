// --- Motor Pin Definitions ---
const int AIN1 = 4;   // Left Motor Direction 1
const int AIN2 = 7;   // Left Motor Direction 2
const int PWMA = 5;   // Left Motor Speed (PWM)
const int BIN1 = 9;   // Right Motor Direction 1
const int BIN2 = 8;   // Right Motor Direction 2
const int PWMB = 6;   // Right Motor Speed (PWM)

// --- Encoder & Tool Pins ---
const int ENCL_A = 2; // Left Encoder Phase A (Interrupt)
const int buzzer = 10;

// --- Precision Constants ---
const int TICKS_PER_CELL = 616; // measured value
const int SLOW_ZONE = 150;      // Ticks before end to start slowing down
volatile long leftTicks = 0;    // Counter for encoder pulses

void setup() {
  Serial.begin(9600);

  // Set Motor Pins as Output
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // Set Encoder Pin
  pinMode(ENCL_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCL_A), countLeft, RISING);

  Serial.println("System Ready. Moving in 5 seconds...");
  delay(5000); // Time to place the robot on the start line
  
  moveOneCell();
}

void countLeft() {
  leftTicks++; // Increment every time the encoder pulses
}

void moveOneCell() {
  leftTicks = 0; // Reset counter for this move
  
  // Set Direction to FORWARD
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  while (leftTicks < TICKS_PER_CELL) {
    // 1. Slow Down Zone
    if (leftTicks > (TICKS_PER_CELL - SLOW_ZONE)) {
      analogWrite(PWMA, 60); // Low power to prevent skidding
      analogWrite(PWMB, 60);
    } 
    // 2. Full Speed Zone
    else {
      analogWrite(PWMA, 120); // Normal cruise speed
      analogWrite(PWMB, 120);
    }
    
  }

  // 3. Precision Brake
  // Setting both pins HIGH acts as an electronic brake
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255); analogWrite(PWMB, 255);
  delay(100); 
  
  // Power Off
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  
  tone(buzzer, 2000, 200); // Beep to signal "Arrived at Center of Cell"
}

void loop() {
  // Do nothing after the move
}
