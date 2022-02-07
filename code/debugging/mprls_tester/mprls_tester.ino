/*!
 * This tests the functionality of the mprls tubed pressure sensor using the Adafruit
 * MRPLS library
 * 
 * Modified from mprls_simpletest.ino by Corinne Smith Feb 7 2022
 * 
 * ///////////////////////////////////////////////////////////////////////////
 * @file mprls_simpletest.ino
 *
 * A basic test of the sensor with default settings
 * 
 * Designed specifically to work with the MPRLS sensor from Adafruit
 * ----> https://www.adafruit.com/products/3965
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 *
 * MIT license, all text here must be included in any redistribution.
 *
 * ///////////////////////////////////////////////////////////////////////////
 */

#include <Wire.h>
#include "Adafruit_MPRLS.h"

Adafruit_MPRLS mpr = Adafruit_MPRLS();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Executing mprls_tester.ino");
  mpr.begin(0x18);

}

void loop() {
  // put your main code here, to run repeatedly:
  float pressure = mpr.readPressure();
  Serial.print("Pressure (hPa): "); Serial.println(pressure);
  Serial.print("Pressure (psi): "); Serial.println(pressure / 68.947572932);
  delay(1000);

}
