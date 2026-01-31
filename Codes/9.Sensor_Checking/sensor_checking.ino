#include <Wire.h>
#include <VL53L0X.h>

// Create sensor objects
VL53L0X sensorL;
VL53L0X sensorF;
VL53L0X sensorR;

// XSHUT Pins (from our previous pin-map)
const int xshutL = 11;
const int xshutF = 12;
const int xshutR = 13;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Reset all sensors by pulling XSHUT low
  pinMode(xshutL, OUTPUT);
  pinMode(xshutF, OUTPUT);
  pinMode(xshutR, OUTPUT);
  
  digitalWrite(xshutL, LOW);
  digitalWrite(xshutF, LOW);
  digitalWrite(xshutR, LOW);
  delay(10);

  // 1. Initialize Left Sensor
  digitalWrite(xshutL, HIGH);
  delay(10);
  if (!sensorL.init()) {
    Serial.println("Failed to detect Left Sensor!");
  } else {
    sensorL.setAddress(0x31);
    sensorL.startContinuous();
    Serial.println("Left Sensor Online (0x31)");
  }

  // 2. Initialize Front Sensor
  digitalWrite(xshutF, HIGH);
  delay(10);
  if (!sensorF.init()) {
    Serial.println("Failed to detect Front Sensor!");
  } else {
    sensorF.setAddress(0x30);
    sensorF.startContinuous();
    Serial.println("Front Sensor Online (0x30)");
  }

  // 3. Initialize Right Sensor
  digitalWrite(xshutR, HIGH);
  delay(10);
  if (!sensorR.init()) {
    Serial.println("Failed to detect Right Sensor!");
  } else {
    sensorR.setAddress(0x32);
    sensorR.startContinuous();
    Serial.println("Right Sensor Online (0x32)");
  }

  Serial.println("\nDistance Readings (mm):");
}

void loop() {
  // Read distances
  int distL = sensorL.readRangeContinuousMillimeters();
  int distF = sensorF.readRangeContinuousMillimeters();
  int distR = sensorR.readRangeContinuousMillimeters();

  // Print results
  Serial.print("Left: "); Serial.print(distL);
  Serial.print("mm | Front: "); Serial.print(distF);
  Serial.print("mm | Right: "); Serial.print(distR);
  Serial.println("mm");

  // Check for errors (8190 usually means out of range or sensor timeout)
  if (sensorL.timeoutOccurred() || sensorF.timeoutOccurred() || sensorR.timeoutOccurred()) {
    Serial.println(" TIMEOUT!");
  }

  delay(200);
}