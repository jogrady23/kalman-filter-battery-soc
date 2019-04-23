/*
 * DESCRIPTION:
 * ------------
 * This program interfaces with the Python Kalman Filter program to collect OCV and current measurements
 *
 * 
 * CONTACT:
 * -----------
 * Jack O'Grady
 * jogrady@usc.edu
 * 
 * © 2019, All Rights Reserved
 */

// INITIALIZE PARAMETERS
// _____________________

// Initialize Sensor Pins
const int currentSensorPin = A0;
const int ocvSensorPin = A1;
const int relayPin = 2;

// Initialize serial output codes
const int OCVMeasurementCode = 1;
const int coulombMeasurementCode = 2;
const int initialOCVMeasurementCode = 3;
const int initialCurrentMeasurementCode = 4;

// Each 18650 Cell has a capacity of 3Ah, which equals 10,800 C. Since 2 cells are in series, capacity is 10,800 C
float totalCoulombs = 10800.0;
float remainingCoulombs = totalCoulombs;

// Power Resistor Value
// The measured value is 1.036 +/- 0.011 Ohms
float Rp = 1.036;

// Determine factor for voltage divider to measure cell OCV
// The measured value of R1 is 980 +/- 1 Ohm
float R1 = 980;
// The measured value of R2 is 977 +/- 1 Ohm
float R2 = 977;
// Using a voltage divider equation, the total voltage is the voltage drop measured on R1 * voltageFactor
float voltageFactor = (R1 + R2) / R1;

// ***FUNCTIONS***
// _______________

// A function used to measure the OCV of the cells
float takeOCVMeasurement() {
  digitalWrite(relayPin, HIGH);
  int count = 0;
  delay(1000); // lets the batteries settle to equilibrium
  float periodVoltageSum = 0;
  while (count < 10000) {
    int rawVoltRead = analogRead(ocvSensorPin);
    float voltage = (rawVoltRead / 1024.0) * 5.0 * voltageFactor;
    periodVoltageSum += voltage;
    count++;
  }
  float voltageAve = periodVoltageSum / count;
  digitalWrite(relayPin, LOW);
  return voltageAve;
}

// A function to print out an OCV measurement to serial
void printOCVMeasurement(int code, float OCV) {
  Serial.print(code);
  Serial.print(',');
  Serial.println(OCV, 3);
}

// A function used to measure the current through R-power
float takeCurrentMeasurement() {
  // The measurement start time is recorded
  unsigned long startTime = millis();
  // 10000 voltage measurements are taken
  int count = 0;
  float periodAmpSum1 = 0;
  while (count < 10000) {
    // Analog readings are mapped to a 5V scale
    int rawVoltRead1 = analogRead(currentSensorPin);
    float voltage1 = (rawVoltRead1 / 1024.0000) * 5.0;
    // Using Ohm's Law, I = V/R
    periodAmpSum1 += voltage1 / Rp;
    count++;
  }
  float ampAve = periodAmpSum1 / count;
  return ampAve;
}

void printCurrentMeasurement(int code, float current, float deltaT) {
  Serial.print(code);
  Serial.print(',');
  Serial.print(current, 4);
  Serial.print(',');
  Serial.println(deltaT);
}

void setup() {
  Serial.begin(9600);
  // Set digital pin to OUTPUT
  pinMode(relayPin, OUTPUT);
}

void loop() {
 
  /*
   * OVERVIEW OF LOOP
   * ----------------
   * - The Arduino waits for Python to communicate via Serial that it needs a measurement
   * - The Arduino then either collects a current or OCV measurement and prints the data to Serial
   *   for Python to read and use in the Kalman Filter program
  */
  char code = ' ';
  if (Serial.available() > 0) {
    code = Serial.read();
    // current code = 1
    if (code=='1') {
      float initialCurrent = takeCurrentMeasurement();
      Serial.println(initialCurrent, 4);
    }
    // OCV code = 2
    else if (code=='2') {
      float initialOCV = takeOCVMeasurement();
      Serial.println(initialOCV, 3);
    }
  }
}
