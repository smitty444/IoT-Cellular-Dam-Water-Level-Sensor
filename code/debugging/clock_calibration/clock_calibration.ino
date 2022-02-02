/*
 * This sketch is used to set the DS3231 real-time clock. Uncomment the proper line of code to set the RTC.
 * Ensure the external 3V coin cell is in place to keep the time after compilation.
 * 
 * Written by Corinne Smith 11-2021 (https://github.com/ARTS-Laboratory/UAV-Deployable-Stage-Height-Sensor/blob/main/V0/V0.4/code/clock_calibration/clock_calibration.ino) 
 */

#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup () {
  Serial.begin(9600);
  rtc.begin();

  // Uncomment to set the date and time of the RTC to when the sketch was compiled
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // Uncomment to set the date and time of the RTC to a user defined time
  // format: rtc.adjust(DateTime(yyyy, mm, dd, hh, mm, ss));
  //rtc.adjust(DateTime(2021, 11, 22, 9, 4, 6));
}

void loop () {
  // determine the current time
  DateTime now = rtc.now();   

  // print the time every second
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  delay(1000);

}
