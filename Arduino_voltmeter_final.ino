/*
 * DESCRIPTION:
 * ------------
 * This program calculates and prints the voltage and measurement number to a Serial Monitor,
 * from which a Python script can scrape the data and write it to a CSV. The program is 
 * intended to be run on an Arduino/Genuino Uno and uses the analog pin A0. 
 * 
 * CONTACT:
 * -----------
 * Jack O'Grady
 * jogrady@usc.edu
 */

const int sensorPin = A0;

// from voltage divider equation
float voltageFactor = 1;

// set initial data point as 1
long measurementNumber = 1;

void setup() {
  Serial.begin(9600);
}

void loop() {
  /*
   * The Arduino reads the analog input from 0 to 1023, so we must convert that to a voltage
   * scale, which is 5V for this board. Once we have that voltage value, we multiply by the
   * voltage factor from the voltage divider in the circuit. We need a voltage divider for
   * voltages greater than 5V as that is the Arduino's max.
   * 
   * The program takes the average of 10000 voltage measurements for each data point. This avoids
   * voltage reading fluctations from the Arduino that were not present on my DMM.
   */
  int count = 0;
  float periodVoltageSum = 0;
  while (count < 10000) {
    int rawVoltRead = analogRead(sensorPin);
    float voltage = (rawVoltRead / 1024.0) * 5.0 * voltageFactor;
    periodVoltageSum += voltage;
    count++;
  }
  float voltageAve = periodVoltageSum / count;

  // prints the output to Serial in a CSV format
  Serial.print(measurementNumber);
  Serial.print(',');
  Serial.println(voltageAve, 3);
  measurementNumber++;
}
