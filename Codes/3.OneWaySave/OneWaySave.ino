#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>

// ================================================================
//           USER CONFIGURATION
// ================================================================
const int TUNING_MODE = 0; 

// *** PHYSICAL CONSTANTS ***
int TICKS_PER_CELL = 585;  
int TICKS_FOR_90   = 280;  
int TICKS_FOR_180  = 609;

// *** PID CONSTANTS ***
float Kp_Wall = 2.0;            
float Kd_Wall = 0.5;            
float Kp_Enc = 1.0; 

// *** SPEED SETTINGS ***
int SEARCH_SPEED = 90;
int FAST_SPEED = 100; // Faster, because we know the path is safe
int CURRENT_SPEED = 90; 
int WALL_DIST = 75; 

// ================================================================
//               HARDWARE SETUP
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
byte visited[16][16]; // Keeps track of cells we ACTUALLY drove on

int curX = 0; int curY = 0; int heading = 0;

enum RobotMode { IDLE, SEARCH_RUN, FAST_RUN };
RobotMode currentMode = IDLE;

// ================================================================
//               INTERRUPTS
// ================================================================
void countLeft() { leftTicks++; }
ISR(PCINT1_vect) { if (digitalRead(ENCR_A) == HIGH) rightTicks++; }
long getAvgTicks() { noInterrupts(); long l = leftTicks; long r = rightTicks; interrupts(); return (l + r) / 2; }
void resetTicks() { noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts(); }

// ================================================================
//               SETUP
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

  initMaze(); // Clears RAM

  if (TUNING_MODE == 1) { for(int i=0; i<3; i++) { moveOneCell(); delay(500); } while(1); }
  else if (TUNING_MODE == 2) { for(int i=0; i<4; i++) { turnInPlace(true, TICKS_FOR_90); delay(500); } while(1); }
  else {
    // *** COMPETITION MODE ***
    if (digitalRead(MODE_SWITCH_PIN) == LOW) {
      // SEARCH MODE (Start Fresh)
      currentMode = SEARCH_RUN;
      CURRENT_SPEED = SEARCH_SPEED;
      // 1 Beep
      tone(buzzer, 1000, 200); delay(250); tone(buzzer, 1500, 400); 
    } else {
      // FAST MODE (Load Map)
      currentMode = FAST_RUN;
      CURRENT_SPEED = FAST_SPEED;
      // 3 Beeps
      for(int i=0; i<3; i++) { tone(buzzer, 2000, 100); delay(150); }
      loadMapFromEEPROM();
      updateFlood(); // Calculates path using STRICT VISITED LOGIC
    }
  }
  delay(1000); 
}

void loop() {
  if (currentMode == SEARCH_RUN) runSearchLogic();
  else if (currentMode == FAST_RUN) runFastLogic();
}

// ================================================================
//               SEARCH LOGIC (One Way: Start -> Center)
// ================================================================
void runSearchLogic() {
  // 1. Mark this cell as SAFE (Visited)
  visited[curX][curY] = 1;

  // 2. Scan Walls
  if (sensorF.readRangeContinuousMillimeters() < 150) setWall(curX, curY, heading);
  if (sensorR.readRangeContinuousMillimeters() < 150) setWall(curX, curY, (heading + 1) % 4);
  if (sensorL.readRangeContinuousMillimeters() < 150) setWall(curX, curY, (heading + 3) % 4);
  
  // 3. Update Path
  updateFlood(); 

  // 4. Check Goal (Center)
  if (dist[curX][curY] == 0) {
    brake();
    
    // *** STOP & SAVE LOGIC (One Way Only) ***
    // We reached the center. We assume the path we took is valid.
    saveMapToEEPROM(); 
    
    // Victory Song
    tone(buzzer, 1000, 100); delay(150); 
    tone(buzzer, 1500, 100); delay(150); 
    tone(buzzer, 2000, 500);
    
    // Freeze here. User must pick up robot and Reset for Fast Run.
    while(1); 
  }
  
  // 5. Navigate
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
//               FAST RUN LOGIC (Strict Visited Mask)
// ================================================================
void runFastLogic() {
  // Goal Check
  if (dist[curX][curY] == 0) {
    brake(); while(1) { tone(buzzer, 3000, 500); delay(1000); }
  }

  int bestDir = -1;
  int minDist = 255;
  int checkOrder[4] = {0, 1, 3, 2}; 
  
  for(int i=0; i<4; i++) {
    int dir = (heading + checkOrder[i]) % 4;
    
    // 1. Check Wall (Map)
    if (!isWall(curX, curY, dir)) {
      int nx = curX, ny = curY;
      if (dir==0) ny++; else if (dir==1) nx++; else if (dir==2) ny--; else if (dir==3) nx--;
      
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
         // 2. STRICT CHECK: Must be VISITED
         // Even if the map says "no wall", if we didn't drive there in Search Mode,
         // we treat it as inaccessible (infinite distance).
         if (visited[nx][ny] == 1) {
            if (dist[nx][ny] < minDist) { minDist = dist[nx][ny]; bestDir = dir; }
         }
      }
    }
  }
  
  // If no visited path is found (shouldn't happen if Search worked), Stop.
  if (bestDir == -1) { brake(); while(1) { tone(buzzer, 500, 500); delay(500); } }
  
  executeMove(bestDir);
}

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
//               ALGORITHM (The Brain)
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
  if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) { walls[nx][ny] |= (1 << ((bdir + 2) % 4)); }
}

int isWall(int bx, int by, int bdir) { return (walls[bx][by] & (1 << bdir)); }

void updateFlood() {
  // Reset Distances
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { dist[i][j] = 255; } }
  
  // Set Target to Center (Standard Flood Fill)
  dist[7][7] = 0; dist[7][8] = 0; dist[8][7] = 0; dist[8][8] = 0;

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
               
               // *** CRITICAL LOGIC FOR FAST RUN ***
               // In Fast Run, we BLOCK the flood fill from entering unvisited cells.
               // This means dist[nx][ny] will remain 255 for any unvisited cell.
               // The robot will naturally only follow the gradient of VISITED cells.
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
  // 1. Save Walls
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { EEPROM.write(addr, walls[i][j]); addr++; } }
  // 2. Save Visited Array
  for(int i=0; i<MAZE_SIZE; i++) { for(int j=0; j<MAZE_SIZE; j++) { EEPROM.write(addr, visited[i][j]); addr++; } }
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
