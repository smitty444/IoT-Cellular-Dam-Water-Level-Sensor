/*
 * Contact Sensor v1.0
 * Parker Lovett 
 * Created: 5/24/2022
 * Last Edit: 5/24/2022
 * Based on template Botletics (Timothy Woo) as well as WHS Sensor (Corinne Smith) 
 *  String of question marks ex. ????????????????????????????? means that these lines are subject to change depending on what sensor is used for the syphon
 */

 //HARDWARE INTERRUPTS FOR MEGA 2, 3, 18, 19, 20, 21 
 //Contact Sensor Pin: 31

 /*SKELETON CODE FOR HARDWARE INTERRUPT INSTEAD OF RTC
 //digitalPinToInterrupt(pin)
 //attachInterrupt(pin)
 //pinMode(pin2, INPUT);
 
 //void pin2Interrupt(void)
 {
  //brings outta sleep
  detachInterrupt(0);
 }

 void enterSleep(void)
 {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();

  //disabling sleep
  sleep_disable();
 }
*/
//libs
#include "Adafruit_FONA.h"            // from botletics: https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
#include "Adafruit_MQTT.h"            // from adafruit:  https://github.com/adafruit/Adafruit_MQTT_Library
#include "Adafruit_MQTT_FONA.h"

#include <Adafruit_MPRLS.h>           // MPRLS (PRESSURE) SENSOR LIBRARY
#include <Adafruit_MCP9808.h>         // MCP9808 (TEMPERATURE) SENSOR LIBRARY 
#include <NewPing.h>                  // JSN-SR04 LIBRARY https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home 
#include <SD.h>                       // SD CARD LIBRARY
#include <SPI.h>                      // SERIAL PERIPHERAL INTERFACE LIBRARY
#include <DS3232RTC.h>                // RTC LIBRARY https://github.com/JChristensen/DS3232RTC
#include <Wire.h>                     // I2C COMMUNICATION LIBRARY
#include <OneWire.h>                  // 1 WIRE COMMUNICATION LIBRARY
#include <DallasTemperature.h>        // EXTERNAL DS18B20 TEMPERATURE SENSOR LIBRARY
#include <Adafruit_INA219.h>          // INA219 CURRENT SENSOR LIBRARY
#include <avr/sleep.h>                // SLEEP LIBRARIES
#include <avr/power.h>

#define SIMCOM_7000                   // cellular MCU we are using

// FONA PINS -----------------------------------------------------------------------------------------
#define FONA_PWRKEY 6
#define FONA_RST 7
#define FONA_DTR 8
#define FONA_TX 10
#define FONA_RX 11

// indicator LEDs
#define redLed 26                      // MQTT publish error
#define yellowLed 24                   // data collection occurring
#define greenLed 23                    // MQTT connected
#define blueLed 22                     // network connected
#define whiteLed 25                    // GPS error
#define interrupt 5                    // interrupt pin for RTC (NOTE: 5 is INT5, the hardware interrupt that is on pin 18)
#define interruptPin 18

//Defining the Analog and Digital pins for the sensor chosen    ?????????????????????????????
#define analog_Pin A0
#define digital_Pin 47 
#define digital_Pin_LED 49

//Defining pins for the Contact Sensor
#define contactPin 31

int sampling_rate = 2;                // initialize the delay between loops. This can be changed with subscribe
const int keepAlive_mins = round(MQTT_CONN_KEEPALIVE / 60);
float accelVal = 0;

// construct the DS18B20
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

//DS3232RTC RTC;

// send AT commands via fona's software serial
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// construct an instance of the LTE fona
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

// MQTT SETUP WITH ADAFRUIT IO -----------------------------------------------------------------------
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "smitty444"
#define AIO_KEY         "aio_UtgK29tCMxzEDaRMLQeGj6wY7P0B"

// pass in fona class and server details to mqtt class
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// THE PUBLISHING FEEDS ------------------------------------------------------------------------------
Adafruit_MQTT_Publish feed_location = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-3.location/csv");
Adafruit_MQTT_Publish feed_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-3.temperature");
Adafruit_MQTT_Publish feed_voltage = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-3.pressure");
Adafruit_MQTT_Publish feed_accelVal = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/syphon-sensor.accelerometer-Val");
// THE SUBSCRIBING FEEDS -----------------------------------------------------------------------------
Adafruit_MQTT_Subscribe feed_deploy = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/syphon-sensor.deploy");

// construct the MCP9808
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

//construct MPRLS
Adafruit_MPRLS mpr = Adafruit_MPRLS(-1, -1);      // Adafruit_MPRLS(RESET_PIN, EOC_PIN)
//global vars
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
char imei[16] = {0};
bool deployed = false;
bool new_time = false;

Adafruit_MQTT_Subscribe feed_sampling_rate = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/syphon-sensor.sampling-rate");
void setup()
{
  Serial.begin(9600);
  Serial.println(F("*** Running: Syphon-Sensor-v1.0 ***"));

  pinMode(interrupt, INPUT_PULLUP);

  mpr.begin();
  ds18b20.begin();
  
  tempsensor.wake();

  // configure the RTC interrupt
  pinMode(interrupt, INPUT_PULLUP);

  //configure the sensor chosen             ?????????????????????????????
  pinMode(analog_Pin, INPUT);
  pinMode(digital_Pin, INPUT);
  pinMode(digital_Pin_LED, OUTPUT);
  
  //configure contact sensor pin
  pinMode(contactPin,INPUT);

  //Begin RTC
  RTC.DS3232RTC::begin();

  //Shield Setup
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH);             // reset is default high

  pinMode(FONA_DTR, OUTPUT);
  digitalWrite(FONA_DTR, LOW);              // initialize to LOW so that if clock is high fona will be awake

  fona.powerOn(FONA_PWRKEY);                // power on fona by pulsing power key

  moduleSetup();                            // establish serial communication, find fona, determine device IMEI

  fona.setFunctionality(1);                 // set fona to its full functionality (AT+CFUN=1)

  fona.setNetworkSettings(F("hologram"));

  // first disable data
  if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable data!"));

  // enable data
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable data, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Enabled data!"));
  
  //SUB FEEDS
  mqtt.subscribe(&feed_deploy);
  mqtt.subscribe(&feed_sampling_rate);

  // set the keepalive interval in seconds
  mqtt.setKeepAliveInterval(600);

  // connect to cell network
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
    digitalWrite(blueLed, LOW);
  }
  Serial.println(F("Connected to cell network!"));
  digitalWrite(blueLed, HIGH);
  readRSSI();

  
  // first ensure that we are finished configuring everything in Adafruit IO and want the package to start collecting data
  while (! deployed) {

    while (!netStatus()) {
      Serial.println(F("Failed to connect to cell network, retrying..."));
      delay(2000); // Retry every 2s
      digitalWrite(blueLed, LOW);
    }

    MQTT_connect();
    // subscription packet subloop, this runs and waits for the toggle switch in Adafruit IO to turn on
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(5000))) {
      if (subscription == &feed_deploy) {
        Serial.print(F("*** Package: ")); Serial.println((char *)feed_deploy.lastread);
        if (strcmp(feed_deploy.lastread, "ON") == 0) {
          Serial.println(F("***Package is deployed"));
          deployed = true;
        }
        else if (strcmp(feed_deploy.lastread, "OFF") == 0) {
          Serial.println(F("***Package not ready"));
          //delay(5000);    // wait five seconds to not spam the serial monitor
        }
      }
      if (subscription == &feed_sampling_rate) {
        Serial.print(F("*** Sampling rate: "));
        Serial.println((char *)feed_sampling_rate.lastread);
        delay(100);
        new_time = true;
      }
    }
    delay(3000);
    
  }
}


void loop()
{
  // wake up the fona (should be done in the interrupt function, add here too for error handling)
  digitalWrite(FONA_DTR, LOW);

  // connect to cell network
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
    digitalWrite(blueLed, LOW);
  }
  Serial.println(F("Connected to cell network!"));
  digitalWrite(blueLed, HIGH);
  readRSSI();

  digitalWrite(yellowLed, HIGH);

  // find the time at which we are logging all this data
  time_t t = RTC.get();

//  Loop code for whatever sensor is decided to use                 ??????????????????????????????????????

  int contactSensorVal = digitalRead(contactPin);
  Serial.print("Contact Sensor Reading: "); Serial.print(contactSensorVal); Serial.println("");
  char contactSensorValBuff[6];
  dtostrf(contactSensorVal, 1, 2, contactSensorValBuff);
  

  //MQTT Subs
    // connect to MQTT 
  MQTT_connect();

  // This is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    // this checks if the time was changed
    if (subscription == &feed_sampling_rate) {
      Serial.print(F("*** Got: "));
      Serial.println((char *)feed_sampling_rate.lastread);
      delay(100);
      new_time = true;
    }
    // reassign the sampling rate
  if (new_time == true) {
    sampling_rate = atoi((char *)feed_sampling_rate.lastread);
    delay(100);
    Serial.print(F("New sampling rate: ")); Serial.println(sampling_rate);
    new_time = false;   // reset the boolean
  }
  //Checking success of the publish feeds
  //MQTT_publish_checkSuccess(feed_temp, tempExBuff);
  //MQTT_publish_checkSuccess(feed_voltage, stageBuff);
  MQTT_publish_checkSuccess(feed_accelVal, contactSensorValBuff);

  //Taking care of the timeout
    Serial.print(F("Waiting for ")); Serial.print(sampling_rate); Serial.println(F(" minutes\r\n"));
  digitalWrite(yellowLed, LOW);

  int remainder = sampling_rate;

  // check if our sampling rate is slower than the server timeout
  if (sampling_rate > keepAlive_mins) {
    int ping_count = sampling_rate / keepAlive_mins;      // number of times we will have to ping per sampling interval
    Serial.print("Ping count: "); Serial.println(ping_count);
    delay(100);
    remainder = sampling_rate % keepAlive_mins;              // alarm frequency between last ping and next data collection cycle
    Serial.print("Remainder value: "); Serial.println(remainder);
    delay(100);
    if (remainder == 0) {
      ping_count--;
      remainder = keepAlive_mins;
    }
    while (ping_count > 0) {     // only run pinging functions when we do not need to sample data
      pingSleep();
      // ping the MQTT broker
      if (!mqtt.ping()) {
        mqtt.disconnect();
      }
      delay(1000);
      ping_count--;
      Serial.print("Ping count: "); Serial.println(ping_count);
    }
  }
  
  Serial.print("Remaining minutes after pings: "); Serial.println(remainder);

  t = RTC.get();
  delay(100);
  if (remainder == keepAlive_mins) {
    int tolerance = 20;
    // set an alarm for the next data collection cycle after pings
    if (minute(t) < 60 - keepAlive_mins) {
      if (second(t) < tolerance) {
        RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 60 - tolerance, minute(t) + keepAlive_mins - 1, 0, 0);
      }
      else {
        RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 0, minute(t) + keepAlive_mins, 0, 0);
      }
    }
    else {
      if (second(t) < tolerance) {
        RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 60 - tolerance, minute(t) - 60 + keepAlive_mins - 1, 0, 0);
      }
      else {
        RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 0, minute(t) - 60 + keepAlive_mins, 0, 0);
      }
    }
  }
  else {
    if (minute(t) < 60 - remainder) {
      RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 0, minute(t) + remainder, 0, 0);
    }
    else {
      RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 0, minute(t) - 60 + remainder, 0, 0);
    }
  }

  RTC.alarm(DS3232RTC::ALARM_1);

  delay(1000);

  goSleep();
  }
}




void pingSleep() {
  Serial.println("Sleeping until next ping");

  // find the current time
  time_t t = RTC.DS3232RTC::get();
  delay(10);

  int tolerance = 20;     // how many seconds we want to make sure the ping has before the mqtt broker times out

  // set an alarm for the next ping required
  if (minute(t) < 60 - keepAlive_mins) {
    if (second(t) < tolerance) {
      RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 60 - tolerance, minute(t) + keepAlive_mins - 1, 0, 0);
    }
    else {
      RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 0, minute(t) + keepAlive_mins, 0, 0);
    }
  }
  else {
    if (second(t) < tolerance) {
      RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 60 - tolerance, minute(t) - 60 + keepAlive_mins - 1, 0, 0);
    }
    else {
      RTC.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 0, minute(t) - 60 + keepAlive_mins, 0, 0);
    }
  }

  RTC.alarm(DS3232RTC::ALARM_1);

  digitalWrite(FONA_DTR, HIGH);
  delay(500);

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

  digitalWrite(FONA_DTR, LOW);
  delay(1000);

  mqtt.ping();
}


void goSleep() {
  Serial.println("Going to sleep...");
  delay(100);

  fona.println("AT+CSCLK=1");
  if (fona.available()) {
    Serial.println(fona.read());
  }

  delay(500);

  digitalWrite(FONA_DTR, HIGH);

  // activate sleep mode, attach interrupt and assign a waking function to run
  sleep_enable();
  attachInterrupt(interrupt, wakeUp, LOW);
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);              // set to full sleep mode
  sleep_cpu();
}

void wakeUp() {
  Serial.println("RTC Interrupt fired");
  delay(100);
  sleep_disable();
  detachInterrupt(interrupt);

  digitalWrite(FONA_DTR, LOW);
  delay(500);

}

/*
float bubble_sort(float a, float b, float c, float d, float e) {
  float sort[window] = {a, b, c, d, e};
  int i, j;
  float temp;

  for (i = 0 ; i < window - 1; i++)
    {
      for (j = 0 ; j < window - i - 1; j++)
        {
          if (sort[j] > sort[j+1])
            {
            // Swap values
            temp = sort[j];
            sort[j] = sort[j+1];
            sort[j+1] = temp;
            }
          }
      }

  return sort[2];
}
*/
void moduleSetup() {
  // this should all take about 3s for the SIMCOM 7000 module
  fonaSS.begin(115200); // Default SIM7000 shield baud rate
  Serial.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    Serial.println(F("FONA not found"));
    while (1); // Don't proceed if it couldn't find the device
  }

  uint8_t type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM800L:
      Serial.println(F("SIM800L")); break;
    case SIM800H:
      Serial.println(F("SIM800H")); break;
    case SIM808_V1:
      Serial.println(F("SIM808 (v1)")); break;
    case SIM808_V2:
      Serial.println(F("SIM808 (v2)")); break;
    case SIM5320A:
      Serial.println(F("SIM5320A (American)")); break;
    case SIM5320E:
      Serial.println(F("SIM5320E (European)")); break;
    case SIM7000:
      Serial.println(F("SIM7000")); break;
    case SIM7070:
      Serial.println(F("SIM7070")); break;
    case SIM7500:
      Serial.println(F("SIM7500")); break;
    case SIM7600:
      Serial.println(F("SIM7600")); break;
    default:
      Serial.println(F("???")); break;
  }


  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
}


bool netStatus() {
  int n = fona.getNetworkStatus();

  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) {
    return false;
    digitalWrite(blueLed, LOW);
  }
  else return true;
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    digitalWrite(greenLed, HIGH);
    return;
  }

  Serial.println("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    digitalWrite(greenLed, LOW);
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  digitalWrite(greenLed, HIGH);
  Serial.println("MQTT Connected!");
}

void MQTT_publish_checkSuccess(Adafruit_MQTT_Publish &feed, const char *feedContent) {
  Serial.println(F("Sending data..."));
  uint8_t txfailures = 0;
  if (! feed.publish(feedContent)) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(redLed, HIGH);
      delay(200);
      digitalWrite(redLed, LOW);
    }
    Serial.println(F("Failed"));
    txfailures++;
  }
  else {
    digitalWrite(redLed, LOW);
    Serial.println(F("OK!"));
    txfailures = 0;
  }
}

void readRSSI() {
  uint8_t n = fona.getRSSI();
  int8_t r;

  Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  Serial.print(r); Serial.println(F(" dBm"));
}
