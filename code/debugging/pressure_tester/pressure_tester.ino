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

//const int pressurePin = A10; 
#define pressurePin A10

void setup() {
  Serial.begin(9600);
  Serial.println("Executing pressure_tester.ino");
  //pinMode(pressurePin, INPUT);
}

void loop() {
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
  
  Serial.println();
  delay(2000);
}
