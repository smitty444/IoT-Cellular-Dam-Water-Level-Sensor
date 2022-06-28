//Parker Lovett
//Created: 6/1/2022 Last Edited: 6/2/2022
//Test to wake from sleep AND send data to Adafruit
//..............RTC is not hooked up yet on this version..................
//WORKS PIN 2 INT0 ON ARDUINO MEGA 2560

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>

//COPIED---------------------------------------------------------------------------------------------------------------------
#include "Adafruit_FONA.h"            // from botletics: https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
#include "Adafruit_MQTT.h"            // from adafruit:  https://github.com/adafruit/Adafruit_MQTT_Library
#include "Adafruit_MQTT_FONA.h"
#include <Adafruit_MCP9808.h>         // MCP9808 (TEMPERATURE) SENSOR LIBRARY 
#include <SD.h>                       // SD CARD LIBRARY
#include <SPI.h>                      // SERIAL PERIPHERAL INTERFACE LIBRARY
#include <DS3232RTC.h>                // RTC LIBRARY https://github.com/JChristensen/DS3232RTC
#include <Wire.h>                     // I2C COMMUNICATION LIBRARY
#include <OneWire.h>                  // 1 WIRE COMMUNICATION LIBRARY
#include <DallasTemperature.h>        // EXTERNAL DS18B20 TEMPERATURE SENSOR LIBRARY
#include <Adafruit_INA219.h>          // INA219 CURRENT SENSOR LIBRARY

//CLOCK OBJ


DS3232RTC RTC;
 //1 hz pulse for the external D-FF 

/*
   SPI PINS FOR MEGA:
      SCK - 13, 52
      MISO - 12, 50
      MOSI - 11, 51
      SS - general GPIO (53 here)
*/

#define SIMCOM_7000                   // cellular MCU we are using

// FONA PINS -------------------------
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
//#define interrupt 5                    // interrupt pin for RTC (NOTE: 5 is INT5, the hardware interrupt that is on pin 18)
//#define interruptPin 18                //


//int sampling_rate = 2;                // initialize the delay between loops. This can be changed with subscribe
//const int keepAlive_mins = round(MQTT_CONN_KEEPALIVE / 60);
//int liquid_level = 0;

// send AT commands via fona's software serial
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// construct an instance of the LTE fona
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

// MQTT SETUP WITH ADAFRUIT IO --------------
#define AIO_SERVER      "io.adafruit.com"
//#define AIO_SERVERPORT  ####
//#define AIO_USERNAME    "YOUR USER"
//#define AIO_KEY         "YOUR KEY"

// pass in fona class and server details to mqtt class
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//************************************ UNCOMMENT FOR FEEDS BELONGING TO SENSOR 1 *****************
// THE PUBLISHING FEEDS ----------------------------
//Adafruit_MQTT_Publish feed_location = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-3.location/csv");
Adafruit_MQTT_Publish feed_fona_lipo = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/syphon-sensor.lipo-battery");
//Adafruit_MQTT_Publish feed_external_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-3.external-temperature");
Adafruit_MQTT_Publish feed_voltage = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/syphon-sensor.package-battery");
Adafruit_MQTT_Publish feed_sensorVal = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/syphon-sensor.syphonbool");

// THE SUBSCRIBING FEEDS ---------------------------
Adafruit_MQTT_Subscribe feed_deploy = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/syphon-sensor.deploy");


// define the SS for SD card
#define chipSelect 53

// construct the MCP9808
//Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// contact sensor define
#define contactSensor 5
//pinMode(contactSensor, INPUT);

/*
// construct the DS18B20
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

//construct RTC
DS3232RTC RTC;
*/

// construct the INA219
//Adafruit_INA219 ina219;

// some global variables
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
char imei[16] = {0};
bool deployed = false;
bool new_time = false;
bool new_loc = false;

//END COPIED---------------------------------------------------------------------------------------------------------------------

int counter = 0;
int n = counter;
int sleepCount = 3;
double contactValTime = 0;
int counterTimes = 0;
int contactVal = 0;
int prevContactVal = 0;
int sendCount = 0;
void setup() {  //--------------------------------------------------------------------------------SETUP----------------------------------
  Serial.begin(9600);
  Serial.println(F("*** Executing Full_Syphon_Sensor v1.0 ***"));
  pinMode(3, INPUT);
  //COPIED---------------------------------------------------------------------------------------------------------------------
  pinMode(redLed, OUTPUT);
  digitalWrite(redLed, LOW);
  pinMode(yellowLed, OUTPUT);
  digitalWrite(yellowLed, LOW);
  pinMode(greenLed, OUTPUT);
  digitalWrite(greenLed, LOW);
  pinMode(blueLed, OUTPUT);
  digitalWrite(blueLed, LOW);
  pinMode(whiteLed, OUTPUT);
  digitalWrite(whiteLed, LOW);

  RTC.DS3232RTC::begin();
  RTC.squareWave(DS3232RTC::SQWAVE_1_HZ);

  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH);             // reset is default high

  pinMode(FONA_DTR, OUTPUT);
  digitalWrite(FONA_DTR, LOW);              // initialize to LOW so that if clock is high fona will be awake

  fona.powerOn(FONA_PWRKEY);                // power on fona by pulsing power key

  moduleSetup();                            // establish serial communication, find fona, determine device IMEI

  fona.setFunctionality(1);                 // set fona to its full functionality (AT+CFUN=1)

  fona.setNetworkSettings(F("hologram"));   // sets APN as 'hologram', used with Hologram SIM card

  //fona.set_eDRX(1,5,0100);                // sets eDRX mode (not supported by T-mobile LTE

  // first disable data
  if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable data!"));

  // enable data
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable data, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Enabled data!"));

  // subscribe to the subscription feeds
  mqtt.subscribe(&feed_deploy);

  // set the keepalive interval in seconds
  //mqtt.setKeepAliveInterval(600);

  // connect to cell network
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
    digitalWrite(blueLed, LOW);
  }
  Serial.println(F("Connected to cell network!"));
  digitalWrite(blueLed, HIGH);
  //Initial Value
  Serial.println("Sending initial value...");                //NOT
  contactVal = digitalRead(3);
  prevContactVal = contactVal;
  //prevContactVal = contactVal;
  //contactVal = 1-contactVal;
  Serial.print("Contact Sensor Value: ");
  Serial.println(contactVal);
  Serial.println("Sending the Value...");
  char contactValBuff[1];                                    //NOT
  dtostrf(contactVal, 1, 1, contactValBuff);                 //NOT
  MQTT_connect();                                            //NOT
  delay(1000);                                               //NOT
  MQTT_publish_checkSuccess(feed_sensorVal, contactValBuff); //NOT

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
    }
  }
    delay(3000);
  //END COPIED-----------------------------------------------------------------------------------------------------------------
  

  //configure interrupt, MEGA can only do LOW interrupts to wake
  //attachInterrupt(digitalPinToInterrupt(2), ISR__Wake, LOW);
  Serial.println("Success Setup");
}
void loop() { //---------------------------------------------------------------------- LOOP ------------------------------------------------------------
/*
//COPIED
  // wake up the fona (should be done in the interrupt function, add here too for error handling)
  digitalWrite(FONA_DTR, LOW);

  // connect to cell network
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
    digitalWrite(blueLed, LOW);
  }
//  Serial.println(F("Connected to cell network!"));
  digitalWrite(blueLed, HIGH);
  //readRSSI();

  digitalWrite(yellowLed, HIGH);
//END COPIED
*/
  delay(1000);
  counter++;
  sleepCount = 3 - (counter-1);       //counts down seconds before arduino enters sleep mode
  contactVal = digitalRead(3);
  Serial.print("Seconds until Sleep: ");
  Serial.print(sleepCount, DEC);
  Serial.println(" second");
  Serial.print("Sensor Value: ");
  Serial.println(contactVal);         //Want this to be 1 all 3 times before the sleep
  if(counter == 3)
  {
    Serial.println("Entering sleep");
    delay(200);
    counter = 0;
    //digitalWrite(FONA_RST, LOW);
    enterSleep();
  }
}
void ISR__Wake() {
  Serial.println("Waking up ISR Successfully called.");
  delay(100);
  sleep_disable();
  contactVal = digitalRead(3);
  if (contactVal == prevContactVal)
  {
    Serial.println("--------No change, back to sleep!--------");
    enterSleep();
  }
  detachInterrupt(digitalPinToInterrupt(2));
  digitalWrite(FONA_DTR, LOW);
  delay(500);
}

void enterSleep(void)
{
  
  /* Setup pin2 as an interrupt and attach handler. */
  //attachInterrupt(digitalPinToInterrupt(2), ISR__Wake, LOW);
  delay(100);
  prevContactVal = contactVal;

  fona.println("AT+CSCLK=1");
  if (fona.available()) {
    Serial.println(fona.read());
  }
  delay(500);
  digitalWrite(FONA_DTR, HIGH);
  sleep_enable();
  attachInterrupt(digitalPinToInterrupt(2), ISR__Wake, LOW);
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  sleep_cpu();
  
  /* The program will continue from here. */
  /* First thing to do is disable sleep. */
  sleep_disable();
  if (contactVal != prevContactVal)
  {
    stuffOuttaSleep();
  }
  else
  {
    Serial.println("--------No change, back to sleep!--------");
    enterSleep();
  }
  //contactVal = digitalRead(3);
  //contactVal = 1-contactVal;    //invert pin value
//  char contactValBuff[1];
//  dtostrf(contactVal, 1, 1, contactValBuff);
  //Serial.println("Woken up success ");
  //Serial.print("Contact Sensor Value ");
  //Serial.println(contactVal); 

}

void stuffOuttaSleep()
{
  Serial.println("stuffOuttaSleep Function Success Called...");
  //pinMode(redLed, OUTPUT);
  //digitalWrite(redLed, LOW);
  //pinMode(yellowLed, OUTPUT);
  //digitalWrite(yellowLed, LOW);
  //pinMode(greenLed, OUTPUT);
  //digitalWrite(greenLed, LOW);
  //pinMode(blueLed, OUTPUT);
  //digitalWrite(blueLed, LOW);
  //pinMode(whiteLed, OUTPUT);
  //digitalWrite(whiteLed, LOW);

  //pinMode(FONA_RST, OUTPUT);
  //digitalWrite(FONA_RST, HIGH);             // reset is default high

  //pinMode(FONA_DTR, OUTPUT);
  //digitalWrite(FONA_DTR, LOW);              // initialize to LOW so that if clock is high fona will be awake

  //fona.powerOn(FONA_PWRKEY);                // power on fona by pulsing power key

  //moduleSetup();                            // establish serial communication, find fona, determine device IMEI

  fona.setFunctionality(1);                 // set fona to its full functionality (AT+CFUN=1)

  fona.setNetworkSettings(F("hologram"));   // sets APN as 'hologram', used with Hologram SIM card

  //fona.set_eDRX(1,5,0100);                // sets eDRX mode (not supported by T-mobile LTE

  // first disable data
  if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable data!"));

  // enable data
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable data, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Enabled data!"));

  MQTT_connect();
  // subscribe to the subscription feeds
  mqtt.subscribe(&feed_deploy);

  // set the keepalive interval in seconds
  //mqtt.setKeepAliveInterval(600);

  // connect to cell network
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
    digitalWrite(blueLed, LOW);
  }
  Serial.println(F("Connected to cell network!"));
  digitalWrite(blueLed, HIGH);
  //Initial Value
  Serial.println("Sending initial value...");                //NOT
  contactVal = digitalRead(3);    //3 pin is actual sensor val
  //prevContactVal = contactVal;
  //contactVal = 1-contactVal;
  Serial.print("Contact Sensor Value: ");
  Serial.println(contactVal);
  Serial.println("Sending the Value...");
  char contactValBuff[1];                                    //NOT
  dtostrf(contactVal, 1, 1, contactValBuff);                 //NOT
  MQTT_connect();                                            //NOT
  delay(1000);                                               //NOT
  MQTT_publish_checkSuccess(feed_sensorVal, contactValBuff); //NOT
  delay(500);
}

void sensorBounceSleep()
{
  Serial.println("False alarm switching: back to sleep");
  enterSleep();
}


//COPIED FUNCTIONS-----------------------------------------------------------------------------------------------------------------
void goSleep() {
  Serial.println("Going to sleep...");
  delay(100);
  mqtt.disconnect();
  //delay(5000);
  digitalWrite(FONA_RST, LOW);
  delay(100);
  digitalWrite(FONA_RST, HIGH);

  fona.println("AT+CSCLK=1");
  if (fona.available()) {
    Serial.println(fona.read());
  }

  delay(500);

  digitalWrite(FONA_DTR, HIGH);

  // activate sleep mode, attach interrupt and assign a waking function to run
  sleep_enable();
  attachInterrupt(0, wakeUp, LOW);
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);              // set to full sleep mode
  sleep_cpu();
}

void wakeUp() {
  Serial.println("RTC Interrupt fired");
  delay(100);
  sleep_disable();
  detachInterrupt(0);

  digitalWrite(FONA_DTR, LOW);
  delay(500);
}

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
//END COPIED FUNCTIONS-----------------------------------------------------------------------------------------------------------------
