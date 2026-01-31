// --- Pin Definitions (As per your working setup) ---
const int ENCL_A = 2; // Left Encoder Phase A (Interrupt)
const int ENCL_B = 3; // Left Encoder Phase B

volatile long leftTicks = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCL_B, INPUT_PULLUP);
  
  // Attach interrupt for the most accurate counting
  attachInterrupt(digitalPinToInterrupt(ENCL_A), countLeft, RISING);
  
  Serial.println("--- 18cm Calibration Mode ---");
  Serial.println("1. Align the robot back-end with a start line.");
  Serial.println("2. Push it slowly to the 18cm mark.");
  Serial.println("3. Note the final 'Total Ticks' value.");
}

void loop() {
  Serial.print("Total Ticks: ");
  Serial.println(leftTicks);
  delay(100);
}

void countLeft() {
  // Simple increment for calibration
  leftTicks++;
}