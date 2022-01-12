

/*
   This sketch tests the functionality of the publish/subscribe protocols of MQTT-based communication with Adafruit IO
   Temperature and humidity will be sent to the dashboard, and an LED will be controlled from the dashboard.
   The deployment boolean loop will be tested and the temperature bounds controlled by the dashboard as well.

   Written by Corinne Smith 10 Jan 2022
   Resources: AdafruitIO_MQTT_Demo.ino by Timothy Woo (https://github.com/botletics/SIM7000-LTE-Shield)
              adafruitio_15_temp_humidity.ino by Adafruit Industries (https://learn.adafruit.com/adafruit-io-basics-temperature-and-humidity/overview)
*/


#include "Adafruit_FONA.h"                // from botletics: https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
#include "Adafruit_MQTT.h"                // from Adafruit: https://github.com/adafruit/Adafruit_MQTT_Library
#include "Adafruit_MQTT_FONA.h"

// DHT22 SENSOR LIBRARIES
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DATA_PIN 2                        // input pin for the DHT22

#define SIMCOM_7000                       // use the proper cellular MCU

// FONA PINS -----------------------------------------------------------------------
#define FONA_PWRKEY 6
#define FONA_RST 7
#define FONA_TX 10
#define FONA_RX 11

// use the fona's software serial for AT commands to be sent
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// construct the LTE fona
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

// MQTT SETUP WITH ADAFRUIT IO ------------------------------------------------------
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "smitty444"
#define AIO_KEY         "aio_UtgK29tCMxzEDaRMLQeGj6wY7P0B"

// pass in FONA class and server details to the MQTT class
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// THE PUBLISHING FEEDS ------------------------------------------------------------
Adafruit_MQTT_Publish feed_temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME, "/feeds/temperature");
Adafruit_MQTT_Publish feed_humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME, "/feeds/humidity");

// THE SUBSCRIBING FEEDS -----------------------------------------------------------
Adafruit_MQTT_Subscribe feed_led_selection = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME, "/feeds/led-selection");
Adafruit_MQTT_Subscribe feed_led_toggle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME, "/feeds/led-control");
Adafruit_MQTT_Subscribe feed_deployment_toggle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME, "/feeds/deployment-toggle");

// construct the DHT22
DHT_Unified dht(DATA_PIN, DHT22);

// some global variables
boolean deployed = false;         // controls when the loop runs based on the Adafruit IO toggle switch
char imei[16] = {0};              // stores the SIM card's IMEI

void setup() {
  Serial.begin(9600);
  Serial.println("Executing fona_ambience_tester.ino");
  pinMode(DATA_PIN, INPUT);
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH);                         // reset is default high

  fona.powerOn(FONA_PWRKEY);                            // power on the fona

  moduleSetup();                                        // establish serial communication, find the fona module, and determine SIM imei

  fona.setFunctionality(1);                             // this runs the AT+CFUN=1 command to the fona serial

  fona.setNetworkSettings(F("hologram"));               // for use with a Hologram SIM card

  //fona.set_eDRX(

  // turn on the GPS (disable in setup during power saving mode)
  while (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying"));
    delay(2000);
  }
  Serial.println(F("GPS enabled"));

  // turn on the GPRS (disable in setup during power saving mode)
  if (!fona.enableGPRS(false)) {                                    // check to ensure data is off in the first place
    Serial.println(F("Failed to disable data"));
  }
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable data, retrying"));
    delay(2000);
  }
  Serial.println("Data enabled");


  // subscribe to the subscription feeds
  mqtt.subscribe(&feed_led_toggle);
  mqtt.subscribe(&feed_led_selection);
  mqtt.subscribe(&feed_deployment_toggle);

}

void loop() {
  // connect to the cell network
  while (!netStatus()) {
    Serial.println(F("Cannot connect to cell network, retrying"));
    delay(2000);
  }
  Serial.println(F("Connected to cell network!"));
  
  MQTT_connect();     
  
  // first ensure that we are finished configuring everything in Adafruit IO and want the package to start collecting data
  while (! deployed) {
    // subscription packet subloop, this runs and waits for the toggle switch in Adafruit IO to turn on
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(5000))) {
      if (subscription == &feed_deployment_toggle) {
        Serial.print(F("Received: ")); Serial.print((char *)feed_deployment_toggle.lastread);
      }
    }
    // based on the received packet, either break from the loop or repeat again every five seconds
    if (strcmp(feed_deployment_toggle.lastread, "ON") == 0) {
      Serial.println(F("Package is deployed, beginning data collection sequence"));
      deployed = true;
    }
    else if (strcmp(feed_deployment_toggle.lastread, "OFF") == 0) {
      Serial.println(F("Package not ready for deployment, please try again"));
      delay(5000);    // wait five seconds to not spam the serial monitor
    }
  }
  
  // set defaults within the code in case nothing is subscribed
  int led = 13;       
  boolean pin_state = true;
  
  // another subscription subloop, this time for the led 
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    // this checks what the led selection is set to 
    if (subscription == &feed_led_selection) {
      Serial.print(F("LED pin selection: "));
      Serial.println((char *)feed_led_selection.lastread);
      led = atoi((char *)feed_led_selection.lastread);
    }
    // this checks if the led is meant to be on or off
    if (subscription == &feed_led_toggle) {
      Serial.print(F("LED pin state: "));
      Serial.println((char *)feed_led_toggle.lastread);
      boolean pin_state = (char *)feed_led_toggle.lastread;
    }
    // this checks if the deployment switch is ever turned off
    if (subscription == &feed_deployment_toggle) {
      Serial.print(F("Received: ")); Serial.print((char *)feed_deployment_toggle.lastread);
      if (strcmp(feed_deployment_toggle.lastread, "OFF") == 0) {
        Serial.println(F("Deployment turned off"));
        deployed = false;
      }
    }
  }

  // configure the chosen pin and write the state
  pinMode(led, OUTPUT);
  digitalWrite(led, pin_state);
  
  // take temperature data
  sensors_event_t temp;
  dht.temperature().getEvent(&temp);
  float celsius = temp.temperature;
  Serial.print(F("Temperature: ")); 
  Serial.print(celsius); Serial.println(F("C"));
  char tempBuff[6];
  dtostrf(celsius, 1, 2, tempBuff);

  // take humidity data
  sensors_event_t hum;
  dht.humidity().getEvent(&hum);
  float humidity = hum.relative_humidity;
  Serial.print(F("Humidity: "));
  Serial.print(humidity); Serial.println("%");
  char humBuff[6];
  dtostrf(humidity, 1, 2, humBuff);

  // publish data to Adafruit IO
  MQTT_publish_checkSuccess(feed_temperature, tempBuff);
  MQTT_publish_checkSuccess(feed_humidity, humBuff);

  // set a delay for the whole loop
  delay(10000);

}

void moduleSetup() {
  // this should all take about 3s for the SIMCOM 7000 module
  fonaSS.begin(115200);                                         // the default baud rate
  Serial.println("Configuring to 9600 baud");
  fonaSS.println("AT+IPR=9600");                                // uses AT commands to slow down baud rate
  delay(100);
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    Serial.println("FONA not found");
    while (1);
  }

  // determine the fona module
  uint8_t type = fona.type();
  if (type == SIM7000) {                                       // in the botletics fona library, SIM7000 is defined at int 7
    Serial.println("Found SIM7000!");
  }
  else {
    Serial.println("Module is not a SIM7000");
    Serial.println(type);
  }

  // find the imei of the SIM card
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("IMEI: "); Serial.println(imei);
  }
}

boolean netStatus() {
  int n = fona.getNetworkStatus();

  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) return false;
  else return true;
}

void MQTT_connect() {

  int8_t ret;

  // check if we are already connected
  if (mqtt.connected()) {
    return;
  }

  Serial.println("Connecting to MQTT... ");

  // retry connection while MQTT is not connected
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  
  Serial.println("MQTT Connected!");
}

void MQTT_publish_checkSuccess(Adafruit_MQTT_Publish &feed, const char *feedContent) {
  int txfailures = 0;
  Serial.println(F("Sending data..."));
  if (! feed.publish(feedContent)) {
    Serial.println(F("Failed"));
    txfailures++;
  }
  else {
    Serial.println(F("OK!"));
    txfailures = 0;
  }
}
