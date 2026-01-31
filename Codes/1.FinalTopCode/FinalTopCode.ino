#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>

// ================================================================
//           PROFESSOR'S MASTER CONFIGURATION
// ================================================================

// *** TUNING MODE SELECTOR ***
// 0 = COMPETITION MODE (Standard Operation)
// 1 = TUNE DISTANCE (Moves 3 cells forward then stops)
// 2 = TUNE 90 TURN (Turns 90 degrees 4 times)
// 3 = TUNE 180 TURN (Turns 180 degrees 2 times)
const int TUNING_MODE = 2; 

// *** PHYSICAL CONSTANTS (Calibrate these!) ***
int TICKS_PER_CELL = 585;  
int TICKS_FOR_90   = 280;  
int TICKS_FOR_180  = 575; // Tune this using Mode 3!

// *** PID CONTROL ***
float Kp_Wall = 2.0;            
float Kd_Wall = 0.5;            
float Kp_Enc  = 1.0; 

// *** SPEED SETTINGS ***
int SEARCH_SPEED = 90;
int FAST_SPEED   = 100; 
int CURRENT_SPEED = 90; 
int WALL_DIST    = 75; 

// ================================================================
//               HARDWARE PIN DEFINITIONS
// ================================================================
const int AIN1 = 4; const int AIN2 = 7; const int PWMA = 5;
const int BIN1 = 9; const int BIN2 = 8; const int PWMB = 6;
const int ENCL_A = 2; const int ENCR_A = A0; 
const int buzzer = 10;
const int MODE_SWITCH_PIN = A3; 
const int xL = 11; const int xF = 12; const int xR = 13;

VL53L0X sensorL; VL53L0X sensorF; VL53L0X sensorR;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Mapping
const int MAZE_SIZE = 16;
byte walls[16][16];       
byte dist[16][16];
byte visited[16][16]; 

int curX = 0; int curY = 0; int heading = 0;

enum RobotMode { IDLE, SEARCH_RUN, FAST_RUN };
RobotMode currentMode = IDLE;
bool searchingReturn = false; 

// ================================================================
//               INTERRUPTS
// ================================================================
void countLeft() { leftTicks++; }
ISR(PCINT1_vect) { if (digitalRead(ENCR_A) == HIGH) rightTicks++; }
long getAvgTicks() { noInterrupts(); long l = leftTicks; long r = rightTicks; interrupts(); return (l + r) / 2; }
void resetTicks() { noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts(); }

// ================================================================
//               SETUP (Updated with Mode 3)
// ================================================================
void setup() {
  Serial.begin(9600);
  Wire.begin(); 

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(buzzer, OUTPUT); pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);

  pinMode(ENCL_A, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENCL_A), countLeft, RISING);
  pinMode(ENCR_A, INPUT_PULLUP); cli(); PCICR |= (1 << PCIE1); PCMSK1 |= (1 << PCINT8); sei();

  pinMode(xL, OUTPUT); pinMode(xF, OUTPUT); pinMode(xR, OUTPUT);
  digitalWrite(xL, LOW); digitalWrite(xF, LOW); digitalWrite(xR, LOW); delay(50);
  digitalWrite(xL, HIGH); delay(10); sensorL.init(); sensorL.setAddress(0x31); sensorL.setTimeout(500);
  digitalWrite(xF, HIGH); delay(10); sensorF.init(); sensorF.setAddress(0x30); sensorF.setTimeout(500);
  digitalWrite(xR, HIGH); delay(10); sensorR.init(); sensorR.setAddress(0x32); sensorR.setTimeout(500);
  sensorL.startContinuous(); sensorF.startContinuous(); sensorR.startContinuous();

  initMaze(); 
  searchingReturn = false; 

  // ============================================================
  //            TUNING MODES
  // ============================================================
  
  if (TUNING_MODE == 1) { 
    // *** MODE 1: DISTANCE TUNE ***
    // Move 3 cells forward
    for(int i=0; i<3; i++) { moveOneCell(); delay(500); } 
    while(1); 
  }
  else if (TUNING_MODE == 2) { 
    // *** MODE 2: 90 DEGREE TURN TUNE ***
    // Square path
    for(int i=0; i<4; i++) { turnInPlace(true, TICKS_FOR_90); delay(500); } 
    while(1); 
  }
  else if (TUNING_MODE == 3) {
    // *** MODE 3: 180 DEGREE U-TURN TUNE ***
    // Perform U-Turn, Wait, Perform U-Turn (Should be back at start heading)
    for(int i=0; i<2; i++) {
        turnInPlace(true, TICKS_FOR_180); 
        delay(1000);
    }
    while(1); // Freeze
  }
  else {
    // *** COMPETITION MODE ***
    if (digitalRead(MODE_SWITCH_PIN) == LOW) {
      currentMode = SEARCH_RUN;
      CURRENT_SPEED = SEARCH_SPEED;
      tone(buzzer, 1000, 200); delay(250); tone(buzzer, 1500, 400); 
    } else {
      currentMode = FAST_RUN;
      CURRENT_SPEED = FAST_SPEED;
      for(int i=0; i<3; i++) { tone(buzzer, 2000, 100); delay(150); }
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
  if (currentMode == SEARCH_RUN) runSearchLogic();
  else if (currentMode == FAST_RUN) runFastLogic();
}

// ================================================================
//               LOGIC: SEARCH RUN (Double-Save)
// ================================================================
void runSearchLogic() {
  visited[curX][curY] = 1;

  if (sensorF.readRangeContinuousMillimeters() < 150) setWall(curX, curY, heading);
  if (sensorR.readRangeContinuousMillimeters() < 150) setWall(curX, curY, (heading + 1) % 4);
  if (sensorL.readRangeContinuousMillimeters() < 150) setWall(curX, curY, (heading + 3) % 4);
  
  updateFlood(); 

  if (dist[curX][curY] == 0) {
    brake();
    
    if (!searchingReturn) {
        // Checkpoint Save (Center)
        saveMapToEEPROM(); 
        tone(buzzer, 2000, 100); delay(100); tone(buzzer, 2000, 100); delay(500);
        searchingReturn = true;
        updateFlood(); 
        return; 
    }
    else {
        // Final Save (Start)
        saveMapToEEPROM();
        tone(buzzer, 1000, 100); delay(150); tone(buzzer, 1500, 100); delay(150); tone(buzzer, 2000, 500);
        while(1); 
    }
  }
  
  int bestDir = -1;
  int minDist = 255;
  int checkOrder[4] = {0, 1, 3, 2}; 
  
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
//               LOGIC: FAST RUN (Strict Visited)
// ================================================================
void runFastLogic() {
  if (dist[curX][curY] == 0) {
    brake(); while(1) { tone(buzzer, 3000, 500); delay(1000); }
  }

  int bestDir = -1;
  int minDist = 255;
  int checkOrder[4] = {0, 1, 3, 2}; 
  
  for(int i=0; i<4; i++) {
    int dir = (heading + checkOrder[i]) % 4;
    if (!isWall(curX, curY, dir)) {
      int nx = curX, ny = curY;
      if (dir==0) ny++; else if (dir==1) nx++; else if (dir==2) ny--; else if (dir==3) nx--;
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
         if (visited[nx][ny] == 1) {
             if (dist[nx][ny] < minDist) { minDist = dist[nx][ny]; bestDir = dir; }
         }
      }
    }
  }
  if (bestDir == -1) { brake(); while(1) { tone(buzzer, 500, 500); delay(500); } }
  executeMove(bestDir);
}

// ================================================================
//               MOTION & HELPERS
// ================================================================
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

void initMaze() {
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { walls[i][j] = 0; dist[i][j] = 255; visited[i][j] = 0; } }
}
void setWall(int bx, int by, int bdir) {
  walls[bx][by] |= (1 << bdir);
  int nx = bx, ny = by;
  if (bdir == 0) ny++; else if (bdir == 1) nx++; else if (bdir == 2) ny--; else if (bdir == 3) nx--;
  if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) { walls[nx][ny] |= (1 << ((bdir + 2) % 4)); }
}
int isWall(int bx, int by, int bdir) { return (walls[bx][by] & (1 << bdir)); }

void updateFlood() {
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { dist[i][j] = 255; } }
  if (!searchingReturn) { dist[7][7] = 0; dist[7][8] = 0; dist[8][7] = 0; dist[8][8] = 0; } 
  else { dist[0][0] = 0; }

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
               if (currentMode == FAST_RUN && visited[nx][ny] == 0) continue;
               if (dist[nx][ny] > nextVal) { dist[nx][ny] = nextVal; changed = 1; }
            }
          }
        }
    }}
  }
}

void saveMapToEEPROM() {
  int addr = 0;
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { EEPROM.write(addr, walls[i][j]); addr++; } }
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { EEPROM.write(addr, visited[i][j]); addr++; } }
}
void loadMapFromEEPROM() {
  int addr = 0;
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { walls[i][j] = EEPROM.read(addr); addr++; } }
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { visited[i][j] = EEPROM.read(addr); addr++; } }
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255); rightSpeed = constrain(rightSpeed, -255, 255);
  if (leftSpeed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, leftSpeed); }
  else                { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA, -leftSpeed); }
  if (rightSpeed >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, rightSpeed); }
  else                 { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); analogWrite(PWMB, -rightSpeed); }
}
void brake() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255); analogWrite(PWMB, 255); delay(100); analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}
void moveOneCell() {
  resetTicks(); float previousError = 0; unsigned long lastMoveTime = millis(); long lastTicks = 0;
  while(getAvgTicks() < TICKS_PER_CELL) {
    if (millis() - lastMoveTime > 200) { if (getAvgTicks() == lastTicks) { brake(); tone(buzzer, 500, 300); setMotors(-80, -80); delay(300); brake(); return; } lastTicks = getAvgTicks(); lastMoveTime = millis(); }
    int dL = sensorL.readRangeContinuousMillimeters(); int dR = sensorR.readRangeContinuousMillimeters();
    float adjustment = 0; bool wallLeft = (dL < 150); bool wallRight = (dR < 150);
    if (wallLeft && wallRight) { float error = dL - dR; adjustment = (error * Kp_Wall) + ((error - previousError) * Kd_Wall); previousError = error; }
    else if (wallLeft) { float error = dL - WALL_DIST; adjustment = (error * Kp_Wall) + ((error - previousError) * Kd_Wall); previousError = error; }
    else if (wallRight) { float error = WALL_DIST - dR; adjustment = (error * Kp_Wall) + ((error - previousError) * Kd_Wall); previousError = error; }
    else { noInterrupts(); long errEnc = leftTicks - rightTicks; interrupts(); adjustment = errEnc * Kp_Enc; }
    setMotors(CURRENT_SPEED - adjustment, CURRENT_SPEED + adjustment);
  }
  brake();
}
void turnInPlace(bool right, int ticks) {
  resetTicks();
  while (getAvgTicks() < ticks) { if (right) setMotors(CURRENT_SPEED, -CURRENT_SPEED); else setMotors(-CURRENT_SPEED, CURRENT_SPEED); }
  brake();
}