#include <Wire.h>
#include <VL53L0X.h>

// ================================================================
//           USER CONFIGURATION & TUNING
// ================================================================

// 0 = MAZE SOLVER MODE (Normal Run)
// 1 = TUNE FORWARD (Moves 3 cells then stops to check distance)
// 2 = TUNE TURN (Turns 90 degrees 4 times to check accuracy)
const int TUNING_MODE = 0; 

// *** PHYSICAL CONSTANTS ***
int TICKS_PER_CELL = 580;  // Adjust this until it moves exactly 180mm
int TICKS_FOR_90   = 280;  // Adjust this until it turns exactly 90 deg
int TICKS_FOR_180  = 609;  

// *** PID CONSTANTS ***
float Kp = 2.0;            
float Kd = 1.0;            
int BASE_SPEED = 90;       
int WALL_DIST = 70;        // mm from wall to center

// ================================================================
//               HARDWARE SETUP
// ================================================================
const int AIN1 = 4; const int AIN2 = 7; const int PWMA = 5;
const int BIN1 = 9; const int BIN2 = 8; const int PWMB = 6;
const int ENCL_A = 2; const int buzzer = 10;
const int xL = 11; const int xF = 12; const int xR = 13;

VL53L0X sensorL; VL53L0X sensorF; VL53L0X sensorR;
volatile long leftTicks = 0;

// RAM-ONLY MAZE MAP 
const int MAZE_SIZE = 16;
byte walls[16][16];       
byte dist[16][16];        

// Robot State
int curX = 0;
int curY = 0;
int heading = 0;          // 0=North, 1=East, 2=South, 3=West

// ================================================================
//               INTERRUPT & INIT
// ================================================================
void countTicks() { leftTicks++; }

void setup() {
  Serial.begin(9600);
  Wire.begin(); 

  // Pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(ENCL_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCL_A), countTicks, RISING);

  // Sensor Initialization
  pinMode(xL, OUTPUT); pinMode(xF, OUTPUT); pinMode(xR, OUTPUT);
  digitalWrite(xL, LOW); digitalWrite(xF, LOW); digitalWrite(xR, LOW);
  delay(50);

  digitalWrite(xL, HIGH); delay(10); sensorL.init(); sensorL.setAddress(0x31); sensorL.setTimeout(500);
  digitalWrite(xF, HIGH); delay(10); sensorF.init(); sensorF.setAddress(0x30); sensorF.setTimeout(500);
  digitalWrite(xR, HIGH); delay(10); sensorR.init(); sensorR.setAddress(0x32); sensorR.setTimeout(500);
  
  sensorL.startContinuous(); sensorF.startContinuous(); sensorR.startContinuous();

  // Initialize Maze
  initMaze();

  // Indicate Ready
  tone(buzzer, 2000, 100); delay(150); tone(buzzer, 2000, 100); delay(1000);

  // *** EXECUTE TUNING MODES IF SELECTED ***
  if (TUNING_MODE == 1) {
    // Move 3 cells forward to test distance accuracy
    moveOneCell(); delay(500);
    moveOneCell(); delay(500);
    moveOneCell(); 
    brake();
    while(1); // Stop forever
  }
  else if (TUNING_MODE == 2) {
    // Turn 90 degrees 4 times to test square
    for(int i=0; i<4; i++) {
       turnInPlace(true, TICKS_FOR_90);
       delay(500);
    }
    brake();
    while(1); // Stop forever
  }
}

// ================================================================
//               MOTION CONTROL
// ================================================================
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, leftSpeed); }
  else                { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, -leftSpeed); }

  if (rightSpeed >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, rightSpeed); }
  else                 { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, -rightSpeed); }
}

void brake() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255); analogWrite(PWMB, 255);
  delay(100);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

void moveOneCell() {
  leftTicks = 0;
  float previousError = 0;
  
  // Reset PID for new cell
  while(leftTicks < TICKS_PER_CELL) {
    int dL = sensorL.readRangeContinuousMillimeters();
    int dR = sensorR.readRangeContinuousMillimeters();
    
    // PID Logic (Only if walls are present)
    float error = 0;
    if (dR < 150)       error = WALL_DIST - dR; 
    else if (dL < 150)  error = dL - WALL_DIST; 
    
    float P = error * Kp;
    float D = (error - previousError) * Kd;
    float adjustment = P + D;
    previousError = error;
    
    setMotors(BASE_SPEED - adjustment, BASE_SPEED + adjustment);
  }
  brake();
}

void turnInPlace(bool right, int ticks) {
  leftTicks = 0;
  while (leftTicks < ticks) {
    if (right) setMotors(BASE_SPEED, -BASE_SPEED);
    else       setMotors(-BASE_SPEED, BASE_SPEED);
  }
  brake();
}

// ================================================================
//               ALGORITHM (The Brain - RAM Only)
// ================================================================
void initMaze() {
  for(int i=0; i<MAZE_SIZE; i++) {
    for(int j=0; j<MAZE_SIZE; j++) {
      walls[i][j] = 0;
      dist[i][j] = 255;
    }
  }
  // Set Center Goals
  dist[7][7] = 0; dist[7][8] = 0; dist[8][7] = 0; dist[8][8] = 0;
}

void setWall(int bx, int by, int bdir) {
  walls[bx][by] |= (1 << bdir);
  
  // Set Neighbor Wall
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
  // Reset Distances
  for(int i=0; i<MAZE_SIZE; i++) {
    for(int j=0; j<MAZE_SIZE; j++) {
      if (!((i==7||i==8) && (j==7||j==8))) dist[i][j] = 255;
    }
  }

  int changed = 1;
  while(changed) {
    changed = 0;
    for(int i=0; i<MAZE_SIZE; i++) {
      for(int j=0; j<MAZE_SIZE; j++) {
        if (dist[i][j] == 255) continue;
        int nextVal = dist[i][j] + 1;
        
        for(int d=0; d<4; d++) {
          if (!isWall(i, j, d)) {
            int nx = i, ny = j;
            if (d==0) ny++; else if (d==1) nx++; else if (d==2) ny--; else if (d==3) nx--;
            
            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
              if (dist[nx][ny] > nextVal) {
                dist[nx][ny] = nextVal;
                changed = 1;
              }
            }
          }
        }
      }
    }
  }
}

// ================================================================
//               MAIN LOOP
// ================================================================
void loop() {
  if (TUNING_MODE != 0) return; 

  // 1. Scan Walls
  if (sensorF.readRangeContinuousMillimeters() < 150) setWall(curX, curY, heading);
  if (sensorR.readRangeContinuousMillimeters() < 150) setWall(curX, curY, (heading + 1) % 4);
  if (sensorL.readRangeContinuousMillimeters() < 150) setWall(curX, curY, (heading + 3) % 4);
  
  // 2. Compute Path
  updateFlood();
  
  // 3. Check Goal
  if (dist[curX][curY] == 0) {
    brake();
    for(int k=0; k<3; k++) { tone(buzzer, 1000, 200); delay(200); }
    while(1); // STOP
  }
  
  // 4. Decide Move
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
  
  // 5. Execute
  if (bestDir != -1) {
    int diff = (bestDir - heading + 4) % 4;
    if (diff == 1)      { turnInPlace(true, TICKS_FOR_90); heading = (heading + 1) % 4; }
    else if (diff == 3) { turnInPlace(false, TICKS_FOR_90); heading = (heading + 3) % 4; }
    else if (diff == 2) { turnInPlace(true, TICKS_FOR_180); heading = (heading + 2) % 4; }
    
    moveOneCell();
    
    // Update Coordinates
    if (heading == 0) curY++;
    else if (heading == 1) curX++;
    else if (heading == 2) curY--;
    else if (heading == 3) curX--;
  }
  
  delay(100); 
}