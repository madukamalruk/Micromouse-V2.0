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
//               TUNING: FIXING THE CORNER HIT
// ================================================================
float Kp = 2.0;            
float Kd = 0.5;            
int BASE_SPEED = 90;       
int MAX_SPEED = 110;       
int WALL_DIST = 70;        
int FRONT_STOP_DIST = 80; 

// *** CRITICAL TUNING FOR TURNS ***
int TICKS_FOR_90 = 292;    
int TICKS_FOR_180 = 609;   
int TICKS_PER_CELL = 565;

// INCREASED THIS: Drive further before turning to avoid hitting the corner
int INTERSECTION_OFFSET = 270; 

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
  
  // Ready Sound
  tone(buzzer, 2000, 100); delay(150);
  tone(buzzer, 2000, 100); delay(1000);
}

// ================================================================
//                      MOTOR ACTIONS
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

void moveBlind(int ticks) {
  leftTicks = 0;
  setMotors(BASE_SPEED, BASE_SPEED); 
  while(leftTicks < ticks) {
    // Safety check mostly disabled here to force the "Offset" move
    // But we keep extreme crash protection
    if(sensorF.readRangeContinuousMillimeters() < 40) break; 
  }
  brake();
}

void turn(bool right, int ticks) {
  leftTicks = 0;
  while (leftTicks < ticks) {
    if (right) setMotors(BASE_SPEED, -BASE_SPEED);
    else       setMotors(-BASE_SPEED, BASE_SPEED);
  }
  brake();
  previousError = 0; 
  delay(200);
}

// ================================================================
//                    MAZE SOLVER LOOP
// ================================================================
void loop() {
  int dL = sensorL.readRangeContinuousMillimeters();
  int dR = sensorR.readRangeContinuousMillimeters();
  int dF = sensorF.readRangeContinuousMillimeters();

  // --- PRIORITY 1: LEFT OPENING (Turn Left) ---
  if (dL > 200) {
    // 1. Alert!
    tone(buzzer, 2500, 150); // High pitch beep!
    brake();
    
    // 2. Drive DEEPER into the intersection to avoid clipping corner
    moveBlind(INTERSECTION_OFFSET); 
    
    // 3. Turn
    turn(false, TICKS_FOR_90); 
    
    // 4. Enter new cell
    moveBlind(TICKS_PER_CELL-200);
    return; 
  }

  // --- PRIORITY 2: FRONT BLOCKED (Turn Right or U-Turn) ---
  if (dF < FRONT_STOP_DIST && dF > 5) {
    brake(); // Stop immediately
    
    if (dR > 200) {
       // Right is Open
       tone(buzzer, 1000, 150); // Medium pitch beep
       turn(true, TICKS_FOR_90); 
    } else {
       // Dead End
       tone(buzzer, 500, 400); // Low pitch long beep
       turn(true, TICKS_FOR_180); 
    }
    
    return; 
  }

  // --- PRIORITY 3: PID STRAIGHT ---
  float error = 0;
  if (dR < 150)       error = WALL_DIST - dR;
  else if (dL < 150)  error = dL - WALL_DIST;
  
  float P = error * Kp;
  float D = (error - previousError) * Kd;
  float adjustment = P + D;
  previousError = error;

  setMotors(BASE_SPEED - adjustment, BASE_SPEED + adjustment);
}