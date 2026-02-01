#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>

// *** TUNING MODE SELECTOR ***
// 0 = COMPETITION MODE (Standard Operation)
// 1 = TUNE DISTANCE (Moves 3 cells forward then stops)
// 2 = TUNE TURN (Turns 90 degrees 4 times)
const int TUNING_MODE = 0; 

// *** PHYSICAL CONSTANTS (Calibrate these in Practice!) ***
int TICKS_PER_CELL = 585;  
int TICKS_FOR_90   = 280;  
int TICKS_FOR_180  = 609;

// *** PID CONTROL ***
float Kp_Wall = 2.0;            
float Kd_Wall = 0.5;            
float Kp_Enc  = 1.0; // Heading assist when no walls are seen

// *** SPEED SETTINGS ***
int SEARCH_SPEED = 90;
int FAST_SPEED   = 100; // Faster speed for the second run
int CURRENT_SPEED = 90; 
int WALL_DIST    = 75; // Target mm from wall

// ================================================================
//               HARDWARE PIN DEFINITIONS
// ================================================================
// Motor Driver
const int AIN1 = 4; const int AIN2 = 7; const int PWMA = 5;
const int BIN1 = 9; const int BIN2 = 8; const int PWMB = 6;

// Encoders
const int ENCL_A = 2;  // Left Encoder (Standard Interrupt INT0)
const int ENCR_A = A0; // Right Encoder (Pin Change Interrupt PCINT1)

// UI & Sensors
const int buzzer = 10;
const int MODE_SWITCH_PIN = A3; // Switch: GND = Search, Open/High = Fast
const int xL = 11; const int xF = 12; const int xR = 13;

VL53L0X sensorL; VL53L0X sensorF; VL53L0X sensorR;

// Global Variables
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Mapping & Navigation
const int MAZE_SIZE = 16;
byte walls[16][16];       
byte dist[16][16];
byte visited[16][16]; // Tracks cells actually driven on (Safety Mask)

int curX = 0; int curY = 0; int heading = 0;

enum RobotMode { IDLE, SEARCH_RUN, FAST_RUN };
RobotMode currentMode = IDLE;

// *** TWO-WAY SEARCH TRACKER ***
// false = Phase 1 (Going to Center)
// true  = Phase 2 (Returning to Start to verify map)
bool searchingReturn = false; 

// ================================================================
//               INTERRUPT SERVICE ROUTINES
// ================================================================
// Standard Interrupt for Left Encoder
void countLeft() { 
  leftTicks++; 
}

// Pin Change Interrupt for Right Encoder (A0)
ISR(PCINT1_vect) { 
  // Trigger on Rising Edge check
  if (digitalRead(ENCR_A) == HIGH) {
    rightTicks++; 
  }
}

// Atomic Tick Reading Helper
long getAvgTicks() { 
  noInterrupts(); 
  long l = leftTicks; 
  long r = rightTicks; 
  interrupts(); 
  return (l + r) / 2; 
}

void resetTicks() { 
  noInterrupts(); 
  leftTicks = 0; 
  rightTicks = 0; 
  interrupts(); 
}

// ================================================================
//               SETUP (BOOT SEQUENCE)
// ================================================================
void setup() {
  Serial.begin(9600);
  Wire.begin(); 

  // --- Pin Modes ---
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(buzzer, OUTPUT); 
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);

  // --- Encoder Init ---
  pinMode(ENCL_A, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(ENCL_A), countLeft, RISING);
  
  pinMode(ENCR_A, INPUT_PULLUP); 
  cli(); // Disable interrupts temporarily
  PCICR |= (1 << PCIE1);    // Enable Pin Change Interrupts for Port C (A0-A5)
  PCMSK1 |= (1 << PCINT8);  // Unmask bit for A0
  sei(); // Enable interrupts

  // --- Sensor Init ---
  pinMode(xL, OUTPUT); pinMode(xF, OUTPUT); pinMode(xR, OUTPUT);
  digitalWrite(xL, LOW); digitalWrite(xF, LOW); digitalWrite(xR, LOW); delay(50);
  
  digitalWrite(xL, HIGH); delay(10); sensorL.init(); sensorL.setAddress(0x31); sensorL.setTimeout(500);
  digitalWrite(xF, HIGH); delay(10); sensorF.init(); sensorF.setAddress(0x30); sensorF.setTimeout(500);
  digitalWrite(xR, HIGH); delay(10); sensorR.init(); sensorR.setAddress(0x32); sensorR.setTimeout(500);
  
  sensorL.startContinuous(); sensorF.startContinuous(); sensorR.startContinuous();

  // --- RAM Init ---
  initMaze(); 
  searchingReturn = false; 

  // --- MODE SELECTION ---
  if (TUNING_MODE == 1) { 
    // Calibration Mode: Distance
    for(int i=0; i<3; i++) { moveOneCell(); delay(500); } 
    while(1); 
  }
  else if (TUNING_MODE == 2) { 
    // Calibration Mode: Turning
    for(int i=0; i<4; i++) { turnInPlace(true, TICKS_FOR_90); delay(500); } 
    while(1); 
  }
  else {
    // COMPETITION MODE
    if (digitalRead(MODE_SWITCH_PIN) == LOW) {
      // *** SEARCH MODE (Switch Closed) ***
      currentMode = SEARCH_RUN;
      CURRENT_SPEED = SEARCH_SPEED;
      // Sound: "Low-High" (Ready to Learn)
      tone(buzzer, 1000, 200); delay(250); tone(buzzer, 1500, 400); 
    } else {
      // *** FAST MODE (Switch Open) ***
      currentMode = FAST_RUN;
      CURRENT_SPEED = FAST_SPEED;
      // Sound: "Beep-Beep-Beep" (Ready to Race)
      for(int i=0; i<3; i++) { tone(buzzer, 2000, 100); delay(150); }
      
      // Load the Verified Map Logic
      loadMapFromEEPROM();
      updateFlood();
    }
  }
  delay(1000); 
}

// ================================================================
//               MAIN LOOP
// ================================================================
void loop() {
  if (currentMode == SEARCH_RUN) {
    runSearchLogic();
  } else if (currentMode == FAST_RUN) {
    runFastLogic();
  }
}

// ================================================================
//               LOGIC: SEARCH RUN (Double-Save)
// ================================================================
void runSearchLogic() {
  // 1. Mark Visited (Safe Path Breadcrumbs)
  visited[curX][curY] = 1;

  // 2. Scan Walls
  if (sensorF.readRangeContinuousMillimeters() < 150) setWall(curX, curY, heading);
  if (sensorR.readRangeContinuousMillimeters() < 150) setWall(curX, curY, (heading + 1) % 4);
  if (sensorL.readRangeContinuousMillimeters() < 150) setWall(curX, curY, (heading + 3) % 4);
  
  // 3. Compute Path (Target depends on Phase)
  updateFlood(); 

  // 4. Check Goals
  if (dist[curX][curY] == 0) {
    brake();
    
    // --- PHASE 1 DONE: Reached Center ---
    if (!searchingReturn) {
        // *** CHECKPOINT SAVE ***
        // Save immediately. If we crash returning, this map is still usable!
        saveMapToEEPROM(); 
        
        // Audio Confirmation
        tone(buzzer, 2000, 100); delay(100); tone(buzzer, 2000, 100);
        delay(500);
        
        // Flip Logic: Target is now Start (0,0)
        searchingReturn = true;
        updateFlood(); // Recalculate distances for return trip
        return; 
    }
    
    // --- PHASE 2 DONE: Returned to Start ---
    else {
        // We made it back! Overwrite with the Verified Map.
        saveMapToEEPROM();
        
        // Victory Song (Long & Loud)
        tone(buzzer, 1000, 100); delay(150); 
        tone(buzzer, 1500, 100); delay(150); 
        tone(buzzer, 2000, 500);
        
        while(1); // Stop. User must Reset for Fast Run.
    }
  }
  
  // 5. Navigate
  int bestDir = -1;
  int minDist = 255;
  int checkOrder[4] = {0, 1, 3, 2}; // Front, Right, Left, Back
  
  for(int i=0; i<4; i++) {
    int dir = (heading + checkOrder[i]) % 4;
    if (!isWall(curX, curY, dir)) {
      int nx = curX, ny = curY;
      if (dir==0) ny++; else if (dir==1) nx++; else if (dir==2) ny--; else if (dir==3) nx--;
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
         if (dist[nx][ny] < minDist) { minDist = dist[nx][ny]; bestDir = dir; }
      }
    }
  }
  
  executeMove(bestDir);
}

// ================================================================
//               LOGIC: FAST RUN (Strict Visited Mask)
// ================================================================
void runFastLogic() {
  // Goal Check
  if (dist[curX][curY] == 0) {
    brake(); while(1) { tone(buzzer, 3000, 500); delay(1000); }
  }

  int bestDir = -1;
  int minDist = 255;
  int checkOrder[4] = {0, 1, 3, 2}; // Front priority
  
  for(int i=0; i<4; i++) {
    int dir = (heading + checkOrder[i]) % 4;
    
    // 1. Check Map Walls (Memory)
    if (!isWall(curX, curY, dir)) {
      int nx = curX, ny = curY;
      if (dir==0) ny++; else if (dir==1) nx++; else if (dir==2) ny--; else if (dir==3) nx--;
      
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
         // 2. STRICT SAFETY: Only enter cells we visited during search
         // This prevents taking unexplored shortcuts that might have walls.
         if (visited[nx][ny] == 1) {
             if (dist[nx][ny] < minDist) { minDist = dist[nx][ny]; bestDir = dir; }
         }
      }
    }
  }
  
  if (bestDir == -1) { brake(); while(1) { tone(buzzer, 500, 500); delay(500); } }
  executeMove(bestDir);
}

// Helper to execute turn and move
void executeMove(int bestDir) {
  if (bestDir != -1) {
    int diff = (bestDir - heading + 4) % 4;
    if (diff == 1)      { turnInPlace(true, TICKS_FOR_90); heading = (heading + 1) % 4; }
    else if (diff == 3) { turnInPlace(false, TICKS_FOR_90); heading = (heading + 3) % 4; }
    else if (diff == 2) { turnInPlace(true, TICKS_FOR_180); heading = (heading + 2) % 4; }
    moveOneCell();
    if (heading == 0) curY++; else if (heading == 1) curX++;
    else if (heading == 2) curY--; else if (heading == 3) curX--;
  }
}

// ================================================================
//               ALGORITHM & MAPPING
// ================================================================
void initMaze() {
  for(int i=0; i<MAZE_SIZE; i++) { 
    for(int j=0; j<MAZE_SIZE; j++) { 
      walls[i][j] = 0; 
      dist[i][j] = 255; 
      visited[i][j] = 0; 
    } 
  }
}

void setWall(int bx, int by, int bdir) {
  walls[bx][by] |= (1 << bdir);
  int nx = bx, ny = by;
  if (bdir == 0) ny++; else if (bdir == 1) nx++; else if (bdir == 2) ny--; else if (bdir == 3) nx--;
  if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) { 
    walls[nx][ny] |= (1 << ((bdir + 2) % 4)); 
  }
}

int isWall(int bx, int by, int bdir) { 
  return (walls[bx][by] & (1 << bdir)); 
}

void updateFlood() {
  // 1. Reset Distances
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { dist[i][j] = 255; } }
  
  // 2. Set Target based on Phase
  if (!searchingReturn) {
     // Phase 1: Target = Center
     dist[7][7] = 0; dist[7][8] = 0; dist[8][7] = 0; dist[8][8] = 0;
  } else {
     // Phase 2: Target = Start
     dist[0][0] = 0;
  }

  // 3. Flood Fill
  int changed = 1;
  while(changed) {
    changed = 0;
    for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) {
        if (dist[i][j] == 255) continue;
        int nextVal = dist[i][j] + 1;
        for(int d=0; d<4; d++) {
          if (!isWall(i, j, d)) {
            int nx = i, ny = j;
            if (d==0) ny++; else if (d==1) nx++; else if (d==2) ny--; else if (d==3) nx--;
            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
               
               // *** FAST RUN SAFETY ***
               // In Fast Run, we BLOCK unvisited cells from the map calculation.
               if (currentMode == FAST_RUN && visited[nx][ny] == 0) continue;

               if (dist[nx][ny] > nextVal) { dist[nx][ny] = nextVal; changed = 1; }
            }
          }
        }
    }}
  }
}

// ================================================================
//               EEPROM (Saving Walls + Visited)
// ================================================================
void saveMapToEEPROM() {
  int addr = 0;
  // 1. Save Walls (Address 0-255)
  for(int i=0; i<MAZE_SIZE; i++) { 
    for(int j=0; j<MAZE_SIZE; j++) { 
      EEPROM.write(addr, walls[i][j]); 
      addr++; 
    } 
  }
  // 2. Save Visited (Address 256-511)
  for(int i=0; i<MAZE_SIZE; i++) { 
    for(int j=0; j<MAZE_SIZE; j++) { 
      EEPROM.write(addr, visited[i][j]); 
      addr++; 
    } 
  }
}

void loadMapFromEEPROM() {
  int addr = 0;
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { walls[i][j] = EEPROM.read(addr); addr++; } }
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { visited[i][j] = EEPROM.read(addr); addr++; } }
}

// ================================================================
//               MOTION CONTROL
// ================================================================
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255); rightSpeed = constrain(rightSpeed, -255, 255);
  if (leftSpeed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, leftSpeed); }
  else                { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA, -leftSpeed); }
  if (rightSpeed >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, rightSpeed); }
  else                 { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); analogWrite(PWMB, -rightSpeed); }
}

void brake() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); 
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255); analogWrite(PWMB, 255); 
  delay(100); 
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

void moveOneCell() {
  resetTicks(); 
  float previousError = 0; 
  unsigned long lastMoveTime = millis(); 
  long lastTicks = 0;

  while(getAvgTicks() < TICKS_PER_CELL) {
    // Stall Protection (200ms Timeout)
    if (millis() - lastMoveTime > 200) { 
       if (getAvgTicks() == lastTicks) { 
          brake(); tone(buzzer, 500, 300); setMotors(-80, -80); delay(300); brake(); 
          return; 
       } 
       lastTicks = getAvgTicks(); lastMoveTime = millis(); 
    }

    int dL = sensorL.readRangeContinuousMillimeters(); 
    int dR = sensorR.readRangeContinuousMillimeters();
    float adjustment = 0; 
    bool wallLeft = (dL < 150); 
    bool wallRight = (dR < 150);

    // PID Logic: Priority to center, then single walls
    if (wallLeft && wallRight) { 
      float error = dL - dR; 
      adjustment = (error * Kp_Wall) + ((error - previousError) * Kd_Wall); 
      previousError = error; 
    }
    else if (wallLeft) { 
      float error = dL - WALL_DIST; 
      adjustment = (error * Kp_Wall) + ((error - previousError) * Kd_Wall); 
      previousError = error; 
    }
    else if (wallRight) { 
      float error = WALL_DIST - dR; 
      adjustment = (error * Kp_Wall) + ((error - previousError) * Kd_Wall); 
      previousError = error; 
    }
    else { 
      // Open Space: Use Encoder Heading Assist
      noInterrupts(); 
      long errEnc = leftTicks - rightTicks; 
      interrupts(); 
      adjustment = errEnc * Kp_Enc; 
    }

    setMotors(CURRENT_SPEED - adjustment, CURRENT_SPEED + adjustment);
  }
  brake();
}

void turnInPlace(bool right, int ticks) {
  resetTicks();
  while (getAvgTicks() < ticks) { 
    if (right) setMotors(CURRENT_SPEED, -CURRENT_SPEED); 
    else       setMotors(-CURRENT_SPEED, CURRENT_SPEED); 
  }
  brake();
}
