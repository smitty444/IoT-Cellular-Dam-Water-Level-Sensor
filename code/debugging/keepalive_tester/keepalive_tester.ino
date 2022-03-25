#include <DS3232RTC.h>                // RTC LIBRARY https://github.com/JChristensen/DS3232RTC
#include <Wire.h>                     // I2C COMMUNICATION LIBRARY
#include <avr/sleep.h>                // SLEEP LIBRARIES
#include <avr/power.h>

#define interrupt 5

int value = 20;                       // define a value up here so we can keep the original sampling rate value after each loop (may need an extra variable in the real code)
int sampling_rate = value;
const int MQTT_CONN_KEEPALIVE = 15;   // MQTT server timeout value 


void setup() {
  Serial.begin(9600);
  Serial.println("Executing keepaliver_tester.ino");
  pinMode(interrupt, INPUT_PULLUP);

  // begin the clock
  RTC.begin();

  // clear alarms, enable interrupts on alarm 1, turn of squarewave
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, true);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);

}

void loop() {
 // sampling_rate = value;
  int remainder = sampling_rate;

  // check if our sampling rate is slower than the server timeout
  if (sampling_rate > MQTT_CONN_KEEPALIVE) {
    int ping_count = round(sampling_rate / MQTT_CONN_KEEPALIVE);      // number of times we will have to ping per sampling interval
    delay(100);
    Serial.print("Ping count: "); Serial.println(ping_count);         
    delay(100);
    remainder = sampling_rate % MQTT_CONN_KEEPALIVE;              // alarm frequency between last ping and next data collection cycle
    delay(100);
    Serial.print("Sampling rate: "); Serial.println(remainder);
    delay(100);
    if (remainder == 0) {
      ping_count--;
      remainder = MQTT_CONN_KEEPALIVE;
    }
    while (ping_count > 0) {     // only run pinging functions when we do not need to sample data
      pingSleep();
      ping_count--;
    }
  }
//  else {
//    remainder = sampling_rate;
//  }

  // find the current time
  time_t t = RTC.get();
  delay(100);
  
  Serial.print("***Beginning alarm at ");
  delay(100);
  Serial.print(String(hour(t)));
  Serial.print(":");
  Serial.print(String(minute(t)));
  Serial.print(":");
  Serial.println(String(second(t)));

  // set an alarm for the next data collection cycle after all pinging is finished
  if (second(t) < 60 - remainder) {
    RTC.setAlarm(ALM1_MATCH_SECONDS, second(t) + remainder, 0, 0, 0);
  }
  else {
    RTC.setAlarm(ALM1_MATCH_SECONDS, second(t) - 60 + remainder, 0, 0, 0);
  }
  RTC.alarm(ALARM_1);

  // data collection sleep function
  goSleep();

}

void pingSleep() {
  Serial.println("Going to ping sleep");

  // find the current time
  time_t t = RTC.get();
  delay(100);

  Serial.print("Beginning ping alarm at ");
  delay(100);
  Serial.print(String(hour(t)));
  Serial.print(":");
  delay(50);
  Serial.print(String(minute(t)));
  Serial.print(":");
  delay(50);
  Serial.println(String(second(t)));
  delay(50);

  // set an alarm for the next ping required
  if (second(t) < 60 - MQTT_CONN_KEEPALIVE) {
    RTC.setAlarm(ALM1_MATCH_SECONDS, second(t) + MQTT_CONN_KEEPALIVE, 0, 0, 0);
  }
  else {
    RTC.setAlarm(ALM1_MATCH_SECONDS, second(t) - 60 + MQTT_CONN_KEEPALIVE, 0, 0, 0);
  }
  RTC.alarm(ALARM_1);

  sleep_enable();
  attachInterrupt(interrupt, pingWake, LOW);        // the wake up function is set to pingWake
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);              // set to full sleep mode
  sleep_cpu();
}

void pingWake() {
  Serial.println("Pinging the server..."); 
  delay(500);
  sleep_disable();
  detachInterrupt(interrupt);                   // clear the interrupt flag

// NOTE: a new time stamp is too computationally heavy for an ISR function
    //  time_t t = RTC.get();
    //  delay(100);
    //
    //  Serial.print("Ping time: ");
    //  delay(50);
    //  Serial.print(String(hour(t)));
    //  Serial.print(":");
    //  delay(50);
    //  Serial.print(String(minute(t)));
    //  Serial.print(":");
    //  delay(50);
    //  Serial.println(String(second(t)));
    //  delay(50);

  // ping the MQTT broker
  //mqtt.ping()
  
}


void goSleep() {
  Serial.println("***Going to sleep...");
  delay(500);

  sleep_enable();
  attachInterrupt(interrupt, wakeUp, LOW);          // set wake up function to normal wakeUp
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);              // set to full sleep mode
  sleep_cpu();
}

void wakeUp() {
  Serial.println("***RTC Interrupt fired");
  delay(1000);
  sleep_disable();
  detachInterrupt(interrupt);                      // clear interrupt flag
//  time_t t = RTC.get();
//
//  Serial.print("***Interrupt time: ");
//  Serial.print(String(hour(t)));
//  Serial.print(":");
//  Serial.print(String(minute(t)));
//  Serial.print(":");
//  Serial.println(String(second(t)));
  
}
