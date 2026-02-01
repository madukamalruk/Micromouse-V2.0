#include <Wire.h>
#include <VL53L0X.h>

// ================================================================
//                      PIN DEFINITIONS
// ================================================================
const int AIN1 = 4; const int AIN2 = 7; const int PWMA = 5; 
const int BIN1 = 9; const int BIN2 = 8; const int PWMB = 6; 
const int ENCL_A = 2; const int buzzer = 10;
const int xL = 11; const int xF = 12; const int xR = 13; 

VL53L0X sensorL; VL53L0X sensorF; VL53L0X sensorR;
volatile long leftTicks = 0;

// ================================================================
//                 TUNING PARAMETERS
// ================================================================
float Kp = 2.0;            // Steering strength
float Kd = 0.5;            // Stability (lowered slightly for smoother turns)
int BASE_SPEED = 70;       // Slower speed for better decisions
int MAX_SPEED = 90;       
int WALL_DIST = 70;        // Target distance from side wall
int FRONT_STOP_DIST = 80; // Stop distance for front wall
int TICKS_FOR_90 = 292;    // YOUR TURN CALIBRATION
int TICKS_FOR_180 = 609;   // Approx double 90 turn

float previousError = 0;

void countTicks() { leftTicks++; }

// ================================================================
//                      SETUP
// ================================================================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(ENCL_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCL_A), countTicks, RISING);
  
  pinMode(xL, OUTPUT); pinMode(xF, OUTPUT); pinMode(xR, OUTPUT);
  digitalWrite(xL, LOW); digitalWrite(xF, LOW); digitalWrite(xR, LOW);
  delay(50);
  
  digitalWrite(xL, HIGH); delay(10); sensorL.init(); sensorL.setAddress(0x31); sensorL.setTimeout(500);
  digitalWrite(xF, HIGH); delay(10); sensorF.init(); sensorF.setAddress(0x30); sensorF.setTimeout(500);
  digitalWrite(xR, HIGH); delay(10); sensorR.init(); sensorR.setAddress(0x32); sensorR.setTimeout(500);
  
  sensorL.startContinuous(); sensorF.startContinuous(); sensorR.startContinuous();
  
  tone(buzzer, 2000, 100); delay(1000); // Quick start
}

// ================================================================
//                      MOTOR CONTROL
// ================================================================
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  if (leftSpeed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, leftSpeed); }
  else                { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, -leftSpeed); }
  
  if (rightSpeed >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, rightSpeed); }
  else                 { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, -rightSpeed); }
}

void brake() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255); analogWrite(PWMB, 255);
  delay(150);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

// --- TURN FUNCTIONS ---
void turn(bool right, int ticks) {
  leftTicks = 0;
  while (leftTicks < ticks) {
    if (right) setMotors(100, -100); // Spin Right
    else       setMotors(-100, 100); // Spin Left
  }
  brake();
  previousError = 0; // Reset PID
  delay(200);        // Wait for sensors to settle
}

// ================================================================
//                 SMART INTERSECTION LOGIC
// ================================================================
void handleIntersection() {
  brake(); // STOP! Wall ahead.
  
  int dL = sensorL.readRangeContinuousMillimeters();
  int dR = sensorR.readRangeContinuousMillimeters();
  
  Serial.print("DECISION -> L:"); Serial.print(dL); Serial.print(" R:"); Serial.println(dR);

  // LOGIC: Check where the open space is (> 200mm)
  
  // 1. If Left is Open, Turn Left
  if (dL > 200) {
    Serial.println("Turning LEFT");
    turn(false, TICKS_FOR_90);
  }
  // 2. If Right is Open, Turn Right
  else if (dR > 200) {
    Serial.println("Turning RIGHT");
    turn(true, TICKS_FOR_90);
  }
  // 3. Dead End (Both sides blocked)
  else {
    Serial.println("DEAD END");
    turn(true, TICKS_FOR_180); // Spin 180
  }
}

// ================================================================
//                    MAIN LOOP (PID + LOGIC)
// ================================================================
void loop() {
  int dL = sensorL.readRangeContinuousMillimeters();
  int dR = sensorR.readRangeContinuousMillimeters();
  int dF = sensorF.readRangeContinuousMillimeters();

  // 1. SAFETY: Stop at Front Wall
  if (dF < FRONT_STOP_DIST && dF > 5) {
    handleIntersection(); // Call the brain!
    return; // Restart loop after turning
  }

  // 2. PID CALCULATION (Keep Going Straight)
  float error = 0;

  // Priority: If we have a Right wall, hug it 
  if (dR < 150) {
    error = WALL_DIST - dR; // Hug Right
  }
  // If no Right wall but Left wall exists, hug Left
  else if (dL < 150) {
    error = dL - WALL_DIST; // Hug Left
  }
  // If in corridor (both walls), center yourself
  else if (dL < 150 && dR < 150) {
    error = dL - dR; 
  }
  
  // 3. APPLY PID
  float P = error * Kp;
  float D = (error - previousError) * Kd;
  float adjustment = P + D;
  previousError = error;

  // Steering Logic:
  // Positive Adj -> Turn Left (Right motor faster)
  // Negative Adj -> Turn Right (Left motor faster)
  int leftMotorSpeed  = BASE_SPEED - adjustment;
  int rightMotorSpeed = BASE_SPEED + adjustment;

  setMotors(leftMotorSpeed, rightMotorSpeed);
}
