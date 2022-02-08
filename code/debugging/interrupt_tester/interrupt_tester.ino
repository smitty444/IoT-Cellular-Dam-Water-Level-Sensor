
#include <Wire.h>
#include <DS3232RTC.h>                  // for the RTC https://github.com/JChristensen/DS3232RTC
#include <avr/sleep.h>                  // for sleep mode

int sampling_rate = 5;

#define interrupt 5                     // corresponds to digital pin 18 on Mega


void setup() {
  Serial.begin(9600);
  pinMode(interrupt, INPUT_PULLUP);

  // begin the RTC
  RTC.begin();

  // initialize alarms and clear any past alarm flags or interrupts
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, true);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);

  // uncomment to set an initial alarm
  //  time_t t = RTC.get();
  //
  //  if(second(t) < 60 - sampling_rate) {
  //    RTC.setAlarm(ALM1_MATCH_SECONDS, second(t) + sampling_rate, 0, 0, 0);
  //  }
  //  else {
  //    RTC.setAlarm(ALM1_MATCH_SECONDS, second(t) - 60 + sampling_rate, 0, 0, 0);
  //  }
  //
  //  RTC.alarm(ALARM_1);

  //  RTC.alarm(ALARM_1);
  //  RTC.squareWave(SQWAVE_NONE);
  //  RTC.alarmInterrupt(ALARM_1, true);

}

void loop() {
  Serial.println("begin");
  Serial.println("data goes here");
  delay(100);
  time_t t = RTC.get();

  if (second(t) < 60 - sampling_rate) {
    RTC.setAlarm(ALM1_MATCH_SECONDS, second(t) + sampling_rate, 0, 0, 0);
  }
  else {
    RTC.setAlarm(ALM1_MATCH_SECONDS, second(t) - 60 + sampling_rate, 0, 0, 0);
  }

  RTC.alarm(ALARM_1);
  goSleep();
}

void goSleep() {
  Serial.println("Going to sleep...");
  delay(100);

  // activate sleep mode, attach interrupt and assign a waking function to run
  sleep_enable();
  attachInterrupt(interrupt, wakeUp, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);              // set to full sleep mode
  sleep_cpu();


}

void wakeUp() {
  Serial.println("RTC Interrupt fired");
  delay(100);
  sleep_disable();
  detachInterrupt(interrupt);
}
