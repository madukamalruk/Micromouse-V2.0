
#include <Wire.h>
#include <VL53L0X.h>

// ==========================
// 1. PIN DEFINITIONS 
// ==========================
const int AIN1 = 4; const int AIN2 = 7; const int PWMA = 5; // Left Motor
const int BIN1 = 9; const int BIN2 = 8; const int PWMB = 6; // Right Motor
const int ENCL_A = 2; // Left Encoder Interrupt
const int BUZZER = 10;

// Sensor XSHUT Pins
const int xL = 11;
const int xF = 12;
const int xR = 13;

// ==========================
// 2. CALIBRATION (Adjusted)
// ==========================
const int TICKS_PER_CELL = 565; 
const int TICKS_FOR_90   = 240; 
const int BASE_SPEED     = 100;
const int TURN_SPEED     = 90;

// Wall Thresholds
// If sensor sees < 180mm, there is a wall. (Increased for safety)
const int WALL_DIST      = 180; 

// ==========================
// 3. MAZE VARIABLES
// ==========================
const byte NORTH = 0; const byte EAST  = 1; 
const byte SOUTH = 2; const byte WEST  = 3;

byte x = 0;          
byte y = 0;          
byte heading = NORTH; 

// Map: Bit 0=N, 1=E, 2=S, 3=W, 4=Visited
byte walls[16][16];
byte cost[16][16];

VL53L0X sensorL, sensorF, sensorR;
volatile long leftTicks = 0;

// ==========================
// 4. SETUP (Matching Normal_checking.ino)
// ==========================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Motor Pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  // Encoder
  pinMode(ENCL_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCL_A), countISR, RISING);

  // Sensor Initialization (EXACTLY from your working code)
  pinMode(xL, OUTPUT); pinMode(xF, OUTPUT); pinMode(xR, OUTPUT);
  digitalWrite(xL, LOW); digitalWrite(xF, LOW); digitalWrite(xR, LOW);
  delay(100);

  // Initialize sensors one by one
  digitalWrite(xL, HIGH); delay(50); 
  if(!sensorL.init()) Serial.println("L fail");
  sensorL.setAddress(0x31);
  
  digitalWrite(xF, HIGH); delay(50); 
  if(!sensorF.init()) Serial.println("F fail");
  sensorF.setAddress(0x30);
  
  digitalWrite(xR, HIGH); delay(50); 
  if(!sensorR.init()) Serial.println("R fail");
  sensorR.setAddress(0x32);
  
  sensorL.startContinuous(); sensorF.startContinuous(); sensorR.startContinuous();
  
  // Initialize Map
  initMaze();

  // Ready Signal
  for(int i=0; i<3; i++) { tone(BUZZER, 1000, 100); delay(500); } // [cite: 76]
  Serial.println(">>> READY <<<");
}

// ==========================
// 5. MAIN LOOP
// ==========================
void loop() {
  // 1. Mark current cell as visited
  walls[x][y] |= 0x10;

  // 2. READ SENSORS (Critical Step)
  updateWalls();

  // 3. STOP if at center
  if ((x == 7 || x == 8) && (y == 7 || y == 8)) {
    brake();
    while(1) { tone(BUZZER, 3000, 500); delay(1000); }
  }

  // 4. CALCULATE PATH
  floodFill();
  byte nextDir = getBestDirection();

  // Debugging Print
  Serial.print("At: "); Serial.print(x); Serial.print(","); Serial.println(y);
  Serial.print("Next Dir: "); Serial.println(nextDir);

  // 5. MOVE
  if (nextDir == heading) {
    // If path is forward, DOUBLE CHECK front wall before moving
    if (sensorF.readRangeContinuousMillimeters() < 80) {
        // PANIC: We thought it was clear, but it's not. Re-read.
        brake();
        updateWalls(); 
        return; // Restart loop
    }
    moveForward();
  } else {
    turnToFace(nextDir);
    // After turning, just stop and let loop restart to check sensors again
    // This is slower but safer for debugging
    delay(200); 
    moveForward();
  }
  
  // 6. Update virtual coordinates
  updateCoordinates();
  delay(200);
}

// ==========================
// 6. MOTION FUNCTIONS
// ==========================
void countISR() { leftTicks++; }

void moveForward() {
  leftTicks = 0;
  
  // Set Motors Forward
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  while (leftTicks < TICKS_PER_CELL) {
    int dF = sensorF.readRangeContinuousMillimeters();
    int dL = sensorL.readRangeContinuousMillimeters();
    int dR = sensorR.readRangeContinuousMillimeters();

    // --- EMERGENCY BRAKE ---
    // If we get too close to a wall (60mm), STOP immediately
    if (dF < 60 && dF > 5) { 
      brake();
      tone(BUZZER, 500, 1000); // Error sound
      Serial.println("EMERGENCY STOP");
      return; 
    }

    // Centering (Proportional)
    int lSpeed = BASE_SPEED;
    int rSpeed = BASE_SPEED;

    if (dL < 60 && dL > 5) { lSpeed += 20; rSpeed -= 20; }
    else if (dR < 60 && dR > 5) { lSpeed -= 20; rSpeed += 20; }

    analogWrite(PWMA, lSpeed);
    analogWrite(PWMB, rSpeed);
  }
  brake();
}

void turnToFace(byte targetDir) {
  int diff = targetDir - heading;
  if (diff == 3) diff = -1;
  if (diff == -3) diff = 1;
  if (diff == -2) diff = 2;

  if (diff == 1) turn90(true);
  else if (diff == -1) turn90(false);
  else if (diff == 2 || diff == -2) { turn90(true); delay(200); turn90(true); }
  
  heading = targetDir;
}

void turn90(bool right) {
  leftTicks = 0;
  if (right) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  }

  while (leftTicks < TICKS_FOR_90) {
    analogWrite(PWMA, TURN_SPEED);
    analogWrite(PWMB, TURN_SPEED);
  }
  brake();
}

void brake() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255); analogWrite(PWMB, 255);
  delay(150);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

// ==========================
// 7. FLOOD FILL LOGIC
// ==========================
void initMaze() {
  for (int i=0; i<16; i++) {
    for (int j=0; j<16; j++) {
      walls[i][j] = 0;
      cost[i][j] = 255; 
    }
  }
}

void updateWalls() {
  int dL = sensorL.readRangeContinuousMillimeters();
  int dF = sensorF.readRangeContinuousMillimeters();
  int dR = sensorR.readRangeContinuousMillimeters();

  // Filter out "8190" (Out of range)
  if (dF > 2000) dF = 2000;
  if (dL > 2000) dL = 2000;
  if (dR > 2000) dR = 2000;

  // IMPORTANT: Debug prints to Serial so you can see what the robot sees
  Serial.print("L:"); Serial.print(dL); 
  Serial.print(" F:"); Serial.print(dF); 
  Serial.print(" R:"); Serial.println(dR);

  if (dF < WALL_DIST) setWall(x, y, heading);
  if (dR < WALL_DIST) setWall(x, y, (heading + 1) % 4);
  if (dL < WALL_DIST) setWall(x, y, (heading + 3) % 4);
}

void setWall(byte cx, byte cy, byte dir) {
  walls[cx][cy] |= (1 << dir);
  
  byte nx = cx, ny = cy;
  if (dir == NORTH) ny++;
  if (dir == EAST)  nx++;
  if (dir == SOUTH) ny--;
  if (dir == WEST)  nx--;
  
  if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16) {
    byte oppDir = (dir + 2) % 4; 
    walls[nx][ny] |= (1 << oppDir);
  }
}

void floodFill() {
  for(int i=0; i<16; i++) for(int j=0; j<16; j++) cost[i][j] = 255;
  cost[7][7] = 0; cost[7][8] = 0; cost[8][7] = 0; cost[8][8] = 0;

  bool changed = true;
  while(changed) {
    changed = false;
    for(byte i=0; i<16; i++) {
      for(byte j=0; j<16; j++) {
        if (cost[i][j] == 255) continue; 
        byte currentCost = cost[i][j];
        
        if (j < 15 && !(walls[i][j] & 1)) { 
             if (cost[i][j+1] > currentCost + 1) { cost[i][j+1] = currentCost + 1; changed = true; }
        }
        if (i < 15 && !(walls[i][j] & 2)) { 
             if (cost[i+1][j] > currentCost + 1) { cost[i+1][j] = currentCost + 1; changed = true; }
        }
        if (j > 0 && !(walls[i][j] & 4)) { 
             if (cost[i][j-1] > currentCost + 1) { cost[i][j-1] = currentCost + 1; changed = true; }
        }
        if (i > 0 && !(walls[i][j] & 8)) { 
             if (cost[i-1][j] > currentCost + 1) { cost[i-1][j] = currentCost + 1; changed = true; }
        }
      }
    }
  }
}

byte getBestDirection() {
  byte bestDir = heading; 
  byte minVal = 255;
  
  if (!(walls[x][y] & 1) && y < 15) { if (cost[x][y+1] < minVal) { minVal = cost[x][y+1]; bestDir = NORTH; } }
  if (!(walls[x][y] & 2) && x < 15) { if (cost[x+1][y] < minVal) { minVal = cost[x+1][y]; bestDir = EAST; } }
  if (!(walls[x][y] & 4) && y > 0) { if (cost[x][y-1] < minVal) { minVal = cost[x][y-1]; bestDir = SOUTH; } }
  if (!(walls[x][y] & 8) && x > 0) { if (cost[x-1][y] < minVal) { minVal = cost[x-1][y]; bestDir = WEST; } }
  
  return bestDir;
}

void updateCoordinates() {
  if (heading == NORTH) y++;
  else if (heading == EAST) x++;
  else if (heading == SOUTH) y--;
  else if (heading == WEST) x--;
}
