/*
   This sketch compares the performance of the pressure sensor between the field test equation generated
   and a linear scaling of 0.5-4.5V to 0-10 psi.
   Pressure sensor pinouts:
          red wire: 5V
          black wire: GND
          green wire: pressurePin (analog signal)

   Written by Corinne Smith Jan 2021
*/

#include <SD.h>
#include <SPI.h>


//const int pressurePin = A10;
#define pressurePin A10
#define button 17
#define pinCS 53

String height;


void setup() {
  Serial.begin(9600);
  Serial.println("Executing pressure_tester.ino");
  pinMode(button, INPUT);
  pinMode(pinCS, OUTPUT);

  if(! SD.begin(pinCS)) {
    Serial.println("SD not found.");
    while(1);
  }

  Serial.println("Type in the water's elevation before pressing the button to collect five data readings.");
}

void loop() {

  float readings[5];
  float voltages[5];
  float readingTotal = 0;
  float readingCount = 0;
  float voltTotal = 0;
  float voltCount = 0;
  float avgRead = 0;
  float avgVolt = 0;

  if (Serial.available()) {
    height = Serial.readStringUntil('\n');
    Serial.print("Elevation: "); Serial.print(height); Serial.println(" ft");
  }
  if (digitalRead(button) == HIGH)
  {

    for (int i = 0; i < 5; i++) {
      // get the raw 10 bit analog reading
      readings[i] = analogRead(pressurePin);
      Serial.print("Reading: "); Serial.println(readings[i]);

      // convert analog reading to a voltage
      voltages[i] = readings[i] / 1023 * 5;
      Serial.print("Voltage: "); Serial.println(voltages[i]);

      // using the field test equation:
      float pressure_1 = voltages[i] * 2.5046 - 0.9938;
      Serial.print("Using field equation: "); Serial.print(pressure_1); Serial.println(" psi");
      float feet_1 = voltages[i] * 5.7831 - 2.2947;
      Serial.print("\t"); Serial.print(feet_1); Serial.println(" ft of water");

      // using linear scaling
      float pressure_2 = 2.5 * voltages[i] - 1.25;
      Serial.print("Using linear scaling: "); Serial.print(pressure_2); Serial.println(" psi");
      float feet_2 = voltages[i] * 5.7724 - 2.8862;
      Serial.print("\t"); Serial.print(feet_2); Serial.println(" ft of water");

      Serial.println();

      if (readings[i] > 0) {
        readingTotal = readingTotal + readings[i];
        readingCount++;
      }

      if (voltages[i] > 0) {
        voltTotal = voltTotal + voltages[i];
        voltCount++;
      }
      delay(2000);
    }
    avgRead = readingTotal / readingCount;
    Serial.print("Average reading: "); Serial.println(avgRead);
    avgVolt = voltTotal / voltCount;
    Serial.print("Average voltage: "); Serial.println(avgVolt);
  }

  File file = SD.open("field_test_feb2.csv", FILE_WRITE);
  if (file) {
    file.print(height); file.print(" ");
    file.print(avgRead); file.print(" ");
    file.print(avgVolt); file.print(" ");
    file.println(" "); file.print(" ");
    file.close();
  }
  delay(1000);
}
