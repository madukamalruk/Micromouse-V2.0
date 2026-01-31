// --- Pin Definitions (Matching your working setup) ---
const int ENCL_A = 2; // Left Encoder Interrupt
const int buzzer = 10;

volatile long turnTicks = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCL_A, INPUT_PULLUP);
  
  // Attach interrupt for high-speed pulse capture
  attachInterrupt(digitalPinToInterrupt(ENCL_A), countTicks, RISING);
  
  pinMode(buzzer, OUTPUT);
  tone(buzzer, 1000, 200);

  Serial.println("--- 90-Degree Turn Calibration ---");
  Serial.println("1. Place robot on a sheet of paper.");
  Serial.println("2. Mark its current heading (0 degrees).");
  Serial.println("3. Rotate the robot MANUALLY exactly 90 degrees in place.");
  Serial.println("4. The number below is your TICKS_FOR_90.");
}

void countTicks() {
  turnTicks++;
}

void loop() {
  Serial.print("Current Turn Ticks: ");
  Serial.println(turnTicks);
  delay(200); // Update frequency
}