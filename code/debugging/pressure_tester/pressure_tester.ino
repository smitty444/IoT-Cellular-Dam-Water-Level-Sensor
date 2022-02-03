/*
 * This sketch compares the performance of the pressure sensor between the field test equation generated
 * and a linear scaling of 0.5-4.5V to 0-10 psi. 
 * Pressure sensor pinouts:
 *        red wire: 5V
 *        black wire: GND
 *        green wire: pressurePin (analog signal)
 * 
 * Written by Corinne Smith Jan 2021
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <DS3232RTC.h>

//const int pressurePin = A10; 
#define pressurePin A10
#define pinCS 53
#define LED 13
#define dataLED 4

void setup() {
  Serial.begin(9600);
  Serial.println("Executing pressure_tester.ino");
  pinMode(pinCS, OUTPUT);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  pinMode(dataLED, OUTPUT);
  digitalWrite(dataLED, LOW);

  if(! SD.begin(pinCS)) {
    Serial.println("cannot find SD");
    while(1);
  }

  RTC.begin();
}

void loop() {
  digitalWrite(dataLED, HIGH);

  time_t t = RTC.get();
  // get the raw 10 bit analog reading
  float reading = analogRead(pressurePin);
  Serial.print("Reading: "); Serial.println(reading);

  // convert analog reading to a voltage
  float voltage = reading/1023*5;
  Serial.print("Voltage: "); Serial.println(voltage);
  
  // using the field test equation:
  float pressure_1 = voltage*2.5046 - 0.9938;
  Serial.print("Using field equation: "); Serial.print(pressure_1); Serial.println(" psi");
  float feet_1 = voltage*5.7831 - 2.2947;
  Serial.print("\t"); Serial.print(feet_1); Serial.println(" ft of water");

  // using linear scaling
  float pressure_2 = 2.5*voltage - 1.25;
  Serial.print("Using linear scaling: "); Serial.print(pressure_2); Serial.println(" psi");
  float feet_2 = voltage*5.7724 - 2.8862;
  Serial.print("\t"); Serial.print(feet_2); Serial.println(" ft of water");

  // using feb 1 experimental data
  float feet_3 = reading*0.027 - 3.0065;
  Serial.print("Using feb 1 experimental data: "); Serial.print(feet_3); Serial.println(" ft of water");

  File file = SD.open("002.csv", FILE_WRITE);
  if(file) {
    // write the time stamp
    file.print(String(month(t)));
    file.print("/");
    file.print(String(day(t)));
    file.print("/");
    file.print(String(year(t)));
    file.print(" ");
    file.print(String(hour(t)));
    file.print(":");
    file.print(String(minute(t)));
    file.print(":");
    file.print(String(second(t)));
    file.print(" ");

    // write the data
    file.print(reading); file.print(" ");
    file.print(voltage); file.print(" ");
    file.print(pressure_1); file.print(" ");
    file.print(feet_1); file.print(" ");
    file.print(pressure_2); file.print(" ");
    file.print(feet_2); file.print(" ");
    file.print(feet_3); file.print(" "); 
    file.println("");
    file.close();
  }
  else {
    Serial.print("Error opening file");
    digitalWrite(LED, HIGH);
  }
  Serial.println();
  digitalWrite(dataLED, LOW);
  delay(2000);
}
