
/* This is an example sketch to send battery, temperature, and GPS location data to
   the cloud via either HTTP GET and POST requests or via MQTT protocol. In this
   sketch we will send to dweet.io, a free cloud API, as well as to ThingsBoard.io,
   a very powerful and free IoT platform that allows you to visualize data on dashboards.

   SETTINGS: You can choose to post only once or to post periodically
   by commenting/uncommenting line 57 ("#define samplingRate 30"). When this line is
   commented out the AVR microcontroller and MCP9808 temperature sensor are put to
   sleep to conserve power, but when the line is being used data will be sent to the
   cloud periodically. This makes it operate like a GPS tracker!

   PROTOCOL: You can use HTTP GET or POST requests and you can change the URL to pretty
   much anything you want. You can also use MQTT to publish data to different feeds
   on Adafruit IO. You can also subscribe to Adafruit IO feeds to command the device
   to do something! In order to select a protocol, simply uncomment a line in the #define
   section below!

   DWEET.IO: To check if the data was successfully sent to dweet, go to
   http://dweet.io/get/latest/dweet/for/{IMEI} and the IMEI number is printed at the
   beginning of the code but can also be found printed on the SIMCOM module itself.

   IoT Example Getting-Started Tutorial: https://github.com/botletics/SIM7000-LTE-Shield/wiki/GPS-Tracker-Example
   GPS Tracker Tutorial Part 1: https://www.instructables.com/id/Arduino-LTE-Shield-GPS-Tracking-Freeboardio/
   GPS Tracker Tutorial Part 2: https://www.instructables.com/id/LTE-Arduino-GPS-Tracker-IoT-Dashboard-Part-2/

   Author: Timothy Woo (www.botletics.com)
   Github: https://github.com/botletics/SIM7000-LTE-Shield
   Last Updated: 3/31/2021
   License: GNU GPL v3.0
*/

#include "Adafruit_FONA.h" // https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
#include <HologramSIMCOM.h> //library download https://github.com/HologramEducation/hologram-SIMCOM
#include <SPI.h>
#include <SD.h>

// You don't need the following includes if you're not using MQTT
// You can find the Adafruit MQTT library here: https://github.com/adafruit/Adafruit_MQTT_Library
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"

#define SIMCOM_7000

#define PROTOCOL_MQTT_AIO         // Adafruit IO

/************************* PIN DEFINITIONS *********************************/
// For botletics SIM7000 shield
#define FONA_PWRKEY 6
#define FONA_RST 7
//#define RST_PIN 45
#define FONA_DTR 8 // Connect with solder jumper
//#define FONA_RI 9 // Need to enable via AT commands
#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX

//For HOLOGRAM connection
#define HOLO_KEY "nK>1H*)x"    //device key found on hologram dashboard website 
//char replybuffer[255]; //SSF- Unclear what this is atm????/////////////////////////////////////////////////////////////////////////////////////

//#define LED 13 // Just for testing if needed!

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

SoftwareSerial *fonaSerial = &fonaSS;  //enables fona serial to be displayed on serial monitor

Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();


/************************* MQTT SETUP *********************************/
// MQTT setup
// For Adafruit IO:
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "SFrazer_DHEC"
#define AIO_KEY         "aio_iBDq21PeCdTgrMoUQTWwqbEBWbMv"

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// How many transmission failures in a row we're OK with before reset
uint8_t txfailures = 0;

/****************************** MQTT FEEDS ***************************************/
// Setup feeds for publishing.
// Notice MQTT paths for Adafruit IO follow the form: <username>/feeds/<feedname>
// Also notice that the combined lat/long "location" feed requires "/csv" in the name
// The Adafruit IO map requires this format: sensor_val, lat, long, altitude
Adafruit_MQTT_Publish feed_location = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/location/csv"); // Group GPS data for AIO map in dashboard
//  Adafruit_MQTT_Publish feed_lat = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/latitude/csv");
//  Adafruit_MQTT_Publish feed_long = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/longitude/csv");
Adafruit_MQTT_Publish feed_distance = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/distance");
Adafruit_MQTT_Publish feed_pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
Adafruit_MQTT_Publish feed_ultrasonicSensorHt = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ultrasonicSensorHt");
Adafruit_MQTT_Publish feed_maxWaterElev = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/maxWaterElev");
Adafruit_MQTT_Publish feed_maxWaterElevCtrl = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/maxWaterElevCtrl"); //controller feeds have to be published to be initiallized on the first loop
Adafruit_MQTT_Publish feed_distanceROT = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/distanceROT");
Adafruit_MQTT_Publish feed_publishFrequency = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/publishFrequency");
Adafruit_MQTT_Publish feed_publishFrequencyCtrl = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/publishFrequencyCtrl"); //controller feeds have to be published to be initiallized on the first loop
Adafruit_MQTT_Publish feed_pressureVolt = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressureVolt");
//  Adafruit_MQTT_Publish feed_speed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/speed");
//  Adafruit_MQTT_Publish feed_head = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/heading");
//  Adafruit_MQTT_Publish feed_alt = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/altitude");
//  Adafruit_MQTT_Publish feed_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
//  Adafruit_MQTT_Publish feed_voltage = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/voltage");

// Setup a feeds for subscribing to changes.
// feeds are to modify the publishing frequency and the 'ultrasonicSensorHt' alert level of the sensor.
Adafruit_MQTT_Subscribe feedIn_publishFrequencyCtrl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/publishFrequencyCtrl");
Adafruit_MQTT_Subscribe feedIn_maxWaterElevCtrl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/maxWaterElevCtrl");

/****************************** OTHER STUFF ***************************************/
// For sleeping the AVR
#include <avr/sleep.h>
#include <avr/power.h>


// The following line is used for applications that require repeated data posting, like GPS trackers
// Comment it out if you only want it to post once, not repeatedly every so often
uint32_t samplingRate = 1000; // The time in between posts, in seconds

// The following line can be used to turn off the shield after posting data. This
// could be useful for saving energy for sparse readings but keep in mind that it
// will take longer to get a fix on location after turning back on than if it had
// already been on. Comment out to leave the shield on after it posts data.
//#define turnOffShield // Turn off shield after posting data


uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
char imei[16] = {0}; // Use this for device ID
uint8_t type;
float latitude, longitude, speed_kph, heading, altitude, second;
uint16_t year;
uint8_t month, day, hour, minute;
uint8_t counter = 0;
//char PIN[5] = "1234"; // SIM card PIN

//Input Variables from Dweet model (Some unused variables may be able to be trimmed)

/////////////NEED TO IMPROVE NAMES OF ALL 'maxWaterElev' AND 'ultrasonicSensorHt' SUB-VARIABLES /////////////////////////////////
///////////// (Sorry I changed these, but lowerBound and upperBound had no meaning. -SSF)/////////////////////////

// set up the ultrasonic sensor
#include <NewPing.h>
#define trigPin 25
#define echoPin 23 //trig and echo pin of ultrasonic sensor
NewPing sonar(trigPin, echoPin, 610); // final argument is the max distance in cm
float distance = 0;

float pressure = 0;   //pressure value read in 10 bit analog
float pressureVolt = 0;
char ultrasonicSensorHtLim[5];
char maxWaterElevLim[5];
char waterHeightSL[5];
char initialPressure[5];    //character strings from questions
uint8_t GPScounter = 0;         //counts failed attempts at GPS location getting
float ultrasonicSensorHtLimInt = 0;   //character to numerical values
float maxWaterElevLimInt = 0;   //^
float depthOfWaterInt = 0;    //^^
float waterHeightSLInt = 0;   //^^^
float initialPressureInt = 0;
//char delaysBetweenPosts[5];  //number of seconds between loop function execution -> Replaced by publishFrequency, below.
float distanceROT = 0;            //distance rate of change
float previousDistance = 0;       //previously measured distance value
char maxROT[5];
float maxROTInt = 0;
bool firstLoop  = true;   //for subscription & publishing of Adafruit IO MQTT 'Control' values.
uint16_t timeBuffer = 0;  //buffer of time, in minutes, since the last successful sensor reading was entered
uint16_t timeROT[5]; //array of ints to store the last five durations between the last five successful sensor readings
float avgROT[5];     //array of floats (yuck) to store the previous 5 distance-change values for ROT smoothing
uint8_t indexROT = 0;   //index for ROT smoothing array

//email variables
uint8_t countSinceEmail; //counter of time (in minutes) since the last email was sent.
char emailFrequencyInput[5]; //defined in setup for the minimum delay between warning emails
uint8_t emailFrequency = 0; //integer of emailFrequencyInput
bool emailRecentlySent = false; //set to true when email is sent, then when countSinceEmail >= emailFrequency, set to false.
char emailMessages[10][101] = {{"Send First Email Test"}, {""}, {""}, {""}, {""}, {""}, {""}, {""}, {""}, {""}};
char addMessage[101] = {""};
uint8_t remainingMessages = 9;


//SD CARD VARIABLES
int countForReopen = 0;
char timePostSD[23];
unsigned long positionInFile = 0;
unsigned long previousPositionInFile = 0;
int freeboardSDStatus = 0;
int countSDFail = 0;
File sensorStoredData;

//URL variables
//char URL[200];  // Make sure this is long enough for your request URL
//char body[100]; // Make sure this is long enough for POST body
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
     headBuff[12], altBuff[12], pVoltBuff[12],
     distBuff[10], pressureBuff[12], distanceROTBuff[12], sdStatusBuff[5],
     publishFrequency[5], publishFrequencyCtrl[5], maxWaterElevCtrl[5]; //duration between MQTT publishing events, in MINUTES. >>--NOTE-->> samplingRate is in thousandths of a second.

//EMAIL INITIALIZATION VARIABLES
HologramSIMCOM Hologram(FONA_TX, FONA_RX, FONA_RST, HOLO_KEY);

void setup() {
  Serial.begin(9600);
  //Serial.println(F("*** SIMCom Module IoT Example ***"));
  Serial.println(F("SC DHEC Water Level Sensor (MQTT & AdafruitIO)"));

  //SETUP PIN TO START LOOP
  pinMode(36, OUTPUT);
  digitalWrite(36, HIGH);   //36 pin set to 5 volts
  pinMode(39, OUTPUT);
  digitalWrite(39, LOW);    //39 set to 0 volts NOTE: DO NOT USE AS GROUND AS NORMAL PINS CANNOT DISSIPATE ALL CURRENT SUPPLIED
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state

  //SENSOR SETUP
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);    //defines ultrasonic sensor I/O

  fona.powerOn(FONA_PWRKEY); // Power on the module
  moduleSetup(); // Establishes first-time serial comm and prints IMEI

  // Unlock SIM card if needed
  // Remember to uncomment the "PIN" variable definition above
  /*
    if (!fona.unlockSIM(PIN)) {
    Serial.println(F("Failed to unlock SIM card"));
    }
  */

  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

  // Configure a GPRS APN, username, and password.
  // You might need to do this to access your network's GPRS/data
  // network.  Contact your provider for the exact APN, username,
  // and password values.  Username and password are optional and
  // can be removed, but APN is required.
  //fona.setNetworkSettings(F("your APN"), F("your username"), F("your password"));
  //fona.setNetworkSettings(F("m2m.com.attz")); // For AT&T IoT SIM card
  //fona.setNetworkSettings(F("telstra.internet")); // For Telstra (Australia) SIM card - CAT-M1 (Band 28)
  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card

  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  //fona.setHTTPSRedirect(true);

  /*
    // Other examples of some things you can set:
    fona.setPreferredMode(38); // Use LTE only, not 2G
    fona.setPreferredLTEMode(1); // Use LTE CAT-M only, not NB-IoT
    fona.setOperatingBand("CAT-M", 12); // AT&T uses band 12
    //  fona.setOperatingBand("CAT-M", 13); // Verizon uses band 13
    fona.enableRTC(true);

    fona.enableSleepMode(true);
    fona.set_eDRX(1, 4, "0010");
    fona.enablePSM(true);

    // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
    fona.setNetLED(true, 2, 64, 3000); // on/off, mode, timer_on, timer_off
    fona.setNetLED(false); // Disable network status LED
  */

  // Perform first-time GPS/GPRS setup if the shield is going to remain on,
  // otherwise these won't be enabled in loop() and it won't work!
#ifndef turnOffShield
  // Enable GPS
  while (!fona.enableGPS(true) && GPScounter < 5) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    GPScounter++;
    delay(2000); // Retry every 2s
  }
  GPScounter = 0;
  Serial.println(F("Turned on GPS!"));
  delay(1000); //wait a sec
#if !defined(SIMCOM_3G) && !defined(SIMCOM_7500) && !defined(SIMCOM_7600)
  // Disable GPRS just to make sure it was actually off so that we can turn it on
  if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS!"));

  // Turn on GPRS
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable GPRS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Enabled GPRS!"));
#endif
#endif

  //Use SIM Card server time in GMT
  fona.enableNTPTimeSync(true);

  // set up what we control
#ifdef PROTOCOL_MQTT_AIO
  mqtt.subscribe(&feedIn_maxWaterElevCtrl); // Only if you're using MQTT
  mqtt.subscribe(&feedIn_publishFrequencyCtrl);
#endif

  //Preliminary questions for the data numbers
  //Finding first height
  float distance = sonar.ping_in();   // returns distance in inches using the New Ping library instead of pulseIn
  Serial.print(F("Initial Ultrasonic Sensor Reading: "));
  Serial.print(distance);
  Serial.println(F(" inches"));

  // turn all of these into Adafruit IO commands?
  flushSerial();
  Serial.println(F("Initial distance between ultrasonic sensor and water in inches (verify with reading above): "));
  readline(ultrasonicSensorHtLim, 4, 20);
  Serial.println(ultrasonicSensorHtLim);
  flushSerial();
  Serial.println(F("Initial pressure of sensor on setup in inches: "));
  readline(initialPressure, 3, 20);
  Serial.println(initialPressure);
  flushSerial();
  Serial.println(F("Current water surface elevation (in feet above sea level): "));
  readline(waterHeightSL, 3, 20);
  Serial.println(waterHeightSL);
  flushSerial();
  Serial.println(F("Maximum allowed water surface elevation (in feet above sea level): "));
  readline(maxWaterElevLim, 3, 20);
  Serial.println(maxWaterElevLim);
  flushSerial();
  Serial.println(F("Maximum elevation rate of change (in inch/hour): "));
  readline(maxROT, 3, 20);
  Serial.println(maxROT);
  flushSerial();
  Serial.println(F("How many minutes between website posts do you want? (Suggest: 3): "));
  readline(publishFrequency, 3, 20);
  Serial.println(publishFrequency);
  Serial.println(F("After the first email, minutes to wait before sending any additional warnings? (Suggest: 30 or 60)"));
  readline(emailFrequencyInput, 3, 20);
  Serial.println(emailFrequencyInput);

  emailFrequency = atoi(emailFrequencyInput);
  samplingRate = atol(publishFrequency) * 60 * 1000; // [Time in minutes] * [60 seconds per minute] * [arduino delays are in 1000ths of a second]

  ultrasonicSensorHtLimInt = atoi(ultrasonicSensorHtLim);

  (publishFrequencyCtrl, publishFrequency);
  strcpy(maxWaterElevCtrl, maxWaterElevLim);
  maxROTInt = atoi(maxROT) / (12 * 60); //maxROT is in inch/hour for simplicity of understanding. rateOfChange() function works in foot/minute, so converting, (X inch * (1ft/12inch))/(1 hr * 60min/1hr) -> X/(12*60) ft/min.
  waterHeightSLInt = atoi(waterHeightSL);
  initialPressureInt = atoi(initialPressure);
  maxWaterElevLimInt = atoi(maxWaterElevLim);
  //      maxWaterElevLimInt = maxWaterElevLimInt + waterHeightSLInt;
  //      maxWaterElevLimInt = maxWaterElevLimInt/12;
  //Serial.println(F("*******AFTER VARIABLES IN SETUP********"));
  //postValuesToFeed();

  // initialize all the values of avgROT & timeROT to 0:
  for (indexROT = 0; indexROT < 4; indexROT++) {
    avgROT[indexROT] = 0;
    timeROT[indexROT] = 0;
  }
  indexROT = 0; //reset the index back to zero.

  sdCardInitialize();      //runs initialize SD function
  positionInFile = sensorStoredData.position();    //gets position number on the text document stored on the SD card.



}

void loop() {
  // Connect to cell network and verify connection
  // If unsuccessful, keep retrying every 2s until a connection is made
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
  }
  GPScounter = 0;
  Serial.println(F("Connected to cell network!"));


  // Trigger sensors.
  ultrasonicSensor();   //runs ultrasonicSensor function included in this program
  pressureSensor();     //runs pressureSensor function in this program.

  // Rate of Change calculation
  rateOfChange();

  ////Suggested Delay
  delay(500); // I found that this helps

  // Turn on GPS if it wasn't on already (e.g., if the module wasn't turned off)
  // does this code send up false positives?
#ifdef turnOffShield
  while (!fona.enableGPS(true) && GPScounter < 5) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    GPScounter++;
    delay(2000); // Retry every 2s
  }
  GPScounter = 0;
  Serial.println(F("Turned on GPS!"));
#endif

  // Get a fix on location, try every 2s
  // Use the top line if you want to parse UTC time data as well, the line below it if you don't care
  while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude, &year, &month, &day, &hour, &minute, &second) && GPScounter < 5) {
    //  while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude) && GPScounter < 5) {
    Serial.println(F("Failed to get GPS location, retrying..."));
    GPScounter++;
    delay(2000); // Retry every 2s
  }
  GPScounter = 0;

  Serial.println(F("Got my location!"));
  Serial.println(F("---------------------"));
  Serial.print(F("Latitude: ")); Serial.println(latitude, 6);
  Serial.print(F("Longitude: ")); Serial.println(longitude, 6);
  Serial.print(F("Speed: ")); Serial.println(speed_kph);
  Serial.print(F("Heading: ")); Serial.println(heading);
  Serial.print(F("Altitude: ")); Serial.println(altitude);
  /*
    // Uncomment this if you care about parsing UTC time
    Serial.print(F("Year: ")); Serial.println(year);
    Serial.print(F("Month: ")); Serial.println(month);
    Serial.print(F("Day: ")); Serial.println(day);
    Serial.print(F("Hour: ")); Serial.println(hour);
    Serial.print(F("Minute: ")); Serial.println(minute);
    Serial.print(F("Second: ")); Serial.println(second);
  */
  Serial.println(F("---------------------"));

  // If the shield was already on, no need to re-enable
#if defined(turnOffShield) && !defined(SIMCOM_3G) && !defined(SIMCOM_7500) && !defined(SIMCOM_7600)
  // Disable GPRS just to make sure it was actually off so that we can turn it on
  if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS!"));

  // Turn on GPRS
  while (!fona.enableGPRS(true) && GPScounter < 5) {
    Serial.println(F("Failed to enable GPRS, retrying..."));
    GPScounter++;
    delay(2000); // Retry every 2s
  }
  GPScounter = 0;
  Serial.println(F("Enabled GPRS!"));
#endif

  // Post something to the web API
  // Construct URL and post the data to the web API

  // Format the floating point numbers
  dtostrf(latitude, 1, 6, latBuff);
  dtostrf(longitude, 1, 6, longBuff);
  dtostrf(speed_kph, 1, 0, speedBuff);
  dtostrf(heading, 1, 0, headBuff);
  dtostrf(altitude, 1, 1, altBuff);
  dtostrf(distance, 1, 2, distBuff); // float_val, min_width, digits_after_decimal, char_buffer
  dtostrf(pressure, 1, 2, pressureBuff); // These turn the integers and floats into strings that can be read by the MQTT commands
  dtostrf(distanceROT, 1, 5, distanceROTBuff);
  dtostrf(freeboardSDStatus, 1, 0, sdStatusBuff);
  dtostrf(pressureVolt, 1, 3, pVoltBuff);

  // Also construct a combined, comma-separated location array
  // (many platforms require this for dashboards, like Adafruit IO):
  sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff); // This could look like "10,33.123456,-85.123456,120.5"



  /////////////////////PRINTING VALUES FOR TROUBLESHOOTING///////////////////////

  // postValuesToFeed();

  ////////////////////////////////////////////////////////////////////////////////

#if defined(PROTOCOL_MQTT_AIO)
  // Let's use MQTT!

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected). See the MQTT_connect
  // function definition further below.

  MQTT_connect();

  ////////////////////I'm absolutely positive that we can do the below data publishing through JSON in a group topic. see:https://io.adafruit.com/api/docs/mqtt.html?cpp#group-topics
  ////////////////////This would likely save a ton of data.

  // Now publish all the data to different feeds!
  // The MQTT_publish_checkSuccess handles repetitive stuff.
  // You can see the function near the end of this sketch.
  // For the Adafruit IO dashboard map we send the combined lat/long buffer
  MQTT_publish_checkSuccess(feed_location, locBuff);
  //  MQTT_publish_checkSuccess(feed_speed, speedBuff); // Included in "location" feed
  //  MQTT_publish_checkSuccess(feed_head, headBuff);
  //  MQTT_publish_checkSuccess(feed_alt, altBuff); // Included in "location" feed
  MQTT_publish_checkSuccess(feed_distance, distBuff);
  MQTT_publish_checkSuccess(feed_pressure, pressureBuff);
  MQTT_publish_checkSuccess(feed_publishFrequency, publishFrequency);
  //  MQTT_publish_checkSuccess(feed_ultrasonicSensorHt, ultrasonicSensorHtLim);
  MQTT_publish_checkSuccess(feed_maxWaterElev, maxWaterElevLim);
  MQTT_publish_checkSuccess(feed_distanceROT, distanceROTBuff);
  MQTT_publish_checkSuccess(feed_pressureVolt, pVoltBuff);

  //// We need to initiallize our controller feeds on the first loop
  if (firstLoop == true) {
    MQTT_publish_checkSuccess(feed_publishFrequencyCtrl, publishFrequency);
    MQTT_publish_checkSuccess(feed_maxWaterElevCtrl, maxWaterElevLim);
  }


  ////This is where we check to see if the controller feeds for publishFrequency and ultrasonicSensorHt

  //////// I think that this can be done with Adafruit IO 'get' on the existing publishFrequency/maxWaterElevLim feeds, which will free up two feeds.

  // This is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &feedIn_publishFrequencyCtrl) {
      Serial.print(F("*** Got feedIn_publishFrequencyCtrl Value: "));
      Serial.println((char *)feedIn_publishFrequencyCtrl.lastread);
      strcpy(publishFrequencyCtrl, feedIn_publishFrequencyCtrl.lastread); //get the new value for the control
      //Serial.print(F("*** Changing delay minutes between posts to: "));Serial.println((char *)feedIn_publishFrequencyCtrl.lastread);
      Serial.print(F("*** Changing delay minutes between posts to: ")); Serial.println(publishFrequencyCtrl);
      strcpy(publishFrequency, publishFrequencyCtrl);
      samplingRate = atol(publishFrequency) * 60 * 1000; // [Time in minutes] * [60 seconds per minute] * [arduino delays are in 1000ths of a second]
    }
    if (subscription == &feedIn_maxWaterElevCtrl) {
      Serial.print(F("*** Got feedIn_maxWaterElevCtrl Value: "));
      Serial.println((char *)feedIn_maxWaterElevCtrl.lastread);
      strcpy(maxWaterElevCtrl, feedIn_maxWaterElevCtrl.lastread);
      //Serial.print(F("*** Changing alert water level elevation to: ")); Serial.println((char *)feedIn_maxWaterElevCtrl.lastread);
      Serial.print(F("*** Changing alert water level elevation to: ")); Serial.println(maxWaterElevCtrl);
      strcpy(maxWaterElevLim, maxWaterElevCtrl);
      maxWaterElevLimInt = atoi(maxWaterElevLim);
    }
  }

#endif

  sdStorageVars();       //runs variable storage and formatting to put in data into the SD card
  sensorStoredData.close();      //SD card must close after every value stored or SD card will not save the data sent to it
  Serial.println(F("Saving and Reopening SD"));
  delay(50);
  sdCardInitialize();      //reopens SD card connection after closing

  ///////////// NOTE ////////////////
  ////Be aware that firstLoop influences various functions and code below this point in the loop() function may act incorrectly.

  sendemail();
  firstLoop = false;
  postValuesToFeed();
  Serial.print(F("Waiting for ")); Serial.print(samplingRate / 1000); Serial.println(F(" seconds\r\n"));
  delay(samplingRate); // Delay

}


void printValueSize(char *varName, char *title) {
  char holding[100];
  sprintf(holding, "%s: %s", title, varName);
  Serial.println(holding);
}


void postValuesToFeed() {
  Serial.println(F("/////////////////////////////VALUES GOING TO FEED/////////////////////////////////"));
  Serial.println(F(""));
  printValueSize(pVoltBuff, "pressure voltage");
  Serial.println(F(""));
  printValueSize(locBuff, "locBuff");
  printValueSize(latBuff, "latBuff");
  printValueSize(longBuff, "longBuff");
  printValueSize(distBuff, "distBuff");
  printValueSize(pressureBuff, "pressureBuff");
  printValueSize(distanceROTBuff, "distanceROTBuff");
  printValueSize(ultrasonicSensorHtLim, "ultrasonicSensorHtLim");
  printValueSize(maxWaterElevLim, "maxWaterElevLim");
  printValueSize(publishFrequency, "publishFrequency");
  Serial.println(F("////////////////////////TROUBLESHOOTING CONTROL VALUES/////////////////////////////"));
  Serial.print(F("samplingRate: ")); Serial.println(samplingRate);
  Serial.print(F("publishFrequency: ")); Serial.println(publishFrequency);
  Serial.print(F("publishFrequencyCtrl: ")); Serial.println(publishFrequencyCtrl);
  Serial.print(F("feedIn_publishFrequencyCtrl.lastread: ")); Serial.println((char *)feedIn_publishFrequencyCtrl.lastread);
  Serial.println(F("******"));
  Serial.print(F("maxWaterElevLim: ")); Serial.println(maxWaterElevLim);
  Serial.print(F("maxWaterElevLimInt: ")); Serial.println(maxWaterElevLimInt);
  Serial.print(F("maxWaterElevCtrl: ")); Serial.println(maxWaterElevCtrl);
  Serial.print(F("feedIn_maxWaterElevCtrl.lastread: ")); Serial.println((char *)feedIn_maxWaterElevCtrl.lastread);

  Serial.println(F("///////////////////////////////////////////////////////////////////////////////////"));
}

void moduleSetup() {
  // SIM7000 takes about 3s to turn on and SIM7500 takes about 15s
  // Press Arduino reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset

  // Software serial:
  fonaSS.begin(115200); // Default SIM7000 shield baud rate

  Serial.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    while (1); // Don't proceed if it couldn't find the device
  }


  type = fona.type();
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

  if (!(n == 1 || n == 5)) return false;
  else return true;
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
#ifdef PROTOCOL_MQTT_AIO
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.println(F("Waiting a moment for modem to fully initialize..."));
  delay(5000);
  Serial.println(F("Connecting to MQTT... "));

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println(F("Retrying MQTT connection in 5 seconds..."));
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println(F("MQTT Connected!"));
}

void MQTT_publish_checkSuccess(Adafruit_MQTT_Publish &feed, const char *feedContent) {
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
#endif

// Turn off the MCU completely. Can only wake up from RESET button
// However, this can be altered to wake up via a pin change interrupt

void MCU_powerDown() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0; // Turn off ADC
  power_all_disable ();  // Power off ADC, Timer 0 and 1, serial interface
  sleep_enable();
  sleep_cpu();
}


void ultrasonicSensor() {       //triggers the ultrasonic sensor
  // Clear the trigPin by setting it LOW:
  if (distance >= 0) { //do not discard previous distance if the existing value is zero
    previousDistance = distance;
  }
  distance = sonar.ping_in();

  ultrasonicSensorHtLimInt = ultrasonicSensorHtLimInt / 12; //initial measurement in inches changed to feet. (ultrasonicSensorHtLim is asked by program to user in setup)
  distance = waterHeightSLInt - (distance - ultrasonicSensorHtLimInt);
  //Serial.println("anticipated water surface elevation");
  //Serial.println(distance);
  ultrasonicSensorHtLimInt = ultrasonicSensorHtLimInt * 12;
}
void pressureSensor()
{
  pressure = analogRead(A10);     //reads 10 bit value from analog 10 pin on arduino board
  pressure = pressure * (5.00 / 1023);      //changes 10 bit value to voltage reading
  pressureVolt = pressure;
  pressure = 0.5 + (4.5 * ((pressure - 0.5) / (4)));
  //Serial.println(pressure);
  pressure = pressure + 0.45; //voltage offset to get value as close to 0.5 as possible for starting voltage at STP.
  pressure = (pressure * 2.5) - 1.25; //changes 0.5-4.5 voltage scale to 0-10 PSI reading via linear approach (y=mx+b)
  pressure = pressure * 27.7076;  //PSI to inches H2O conversion
  pressure = pressure / 12;     //inches H2O to feet H2O
  initialPressureInt = initialPressureInt / 12; //initial pressure inches to feet
  pressure = waterHeightSLInt + (pressure - initialPressureInt);  //equation to turn into above sea level
  //pressure = 10*(pressure - 0.5)/(4);
  //Serial.println(pressure);     //reads in 0-1023 bit values and turns it into voltage, calibrates standard atm to 0 value, then changes it to inH2O. THESE NUMBERS WILL VARY ON VOLTAGE RANGE OF PRESSURE SENSOR AND CALIBRATION OF 0.
}

void rateOfChange()
{
  //////ORIGINAL FUNCTION: Misbehaves on bad sensor readings, oversends data.
  //
  //  distanceROT = ((distance-previousDistance)*60)/(samplingRate/1000);
  //  //distanceROT = ((distance-previousDistance)*60)/(samplingRate);
  //  if (distanceROT > maxROTInt)
  //  {
  //    //sendemail();
  //  }

  //////NEW FUNCTION PSEUDOCODE:
  //create variables for sum.
  float sumAvgROT = 0;
  uint16_t sumTimeROT = 0;

  //Checks if distance is >= maximum allowed. If so, send email (msgID = 1)

  if (distance >= maxWaterElevLimInt) {
    strcpy(addMessage, sprintf("(UTC %s/%s,%s:%s)Water Level is: %s, exceeds max elevation of: %s\r\n", month, day, hour, minute, distance, maxWaterElevLimInt));
    addEmailMessage();
  }

  //If distance <= 0, this is expected to mean that the sensor had a bad reading. So, do not add this data to the running ROT average count, but add the minute value of samplingRate to the failed time-buffer value.
  //time buffer acts to correctly show the time duration for ROT smoothing in case of a failed sensor reading.

  else if (distance <= 0) {
    timeBuffer = timeBuffer + (samplingRate / 60000);
  }

  else {
    //set first value of avgROT for smoothing.
    avgROT[indexROT] = distance - previousDistance;
    timeROT[indexROT] = timeBuffer + (samplingRate / 60000);
    //reset timeBuffer after a successful read
    timeBuffer = 0;
  }

  //loop through and sum avgROT and timeROT into sumAvgROT and sumTimeROT respectively.
  for (int sumloop = 0; sumloop < 5; sumloop++) {
    sumAvgROT = sumAvgROT + avgROT[sumloop];
    sumTimeROT = sumTimeROT + timeROT[sumloop];
  }

  distanceROT = sumAvgROT / sumTimeROT; //the rate over time change (rate of change) averaged per minute over the last five sensor readings & durations.

  if ( distanceROT > maxROTInt || (-1 * distanceROT) > maxROTInt ) { //check both, considering absolute value of rate of change, the abs()/fabs() functions seemed to be behaving unusually.
    strcpy(addMessage, sprintf("(UTC %s/%s,%s:%s)Water Rate of Change is: %s ft/hr, exceeds max rate of: %s ft/hr.\r\n", month, day, hour, minute, distanceROT * 60, maxROTInt * 60));
    addEmailMessage();
  }

}


void addEmailMessage() { //adds the current contents of addMessage char array to the emailMessages array, based on the remainingMessages count.
  if ( remainingMessages > 0 ) {
    sprintf(emailMessages[10 - remainingMessages], "%s\r\n", addMessage); //set the lowest available message slot of the 10 available to the value of addMessage and add carriage returns and character returns
    remainingMessages--; //decrease the number of messages remaining counter.
  }
  strcpy(addMessage, ""); //reset the value of addMessage to blank-ish (not completely blank).
}

void sendemail()  //IMPORTANT: DOES NOT SEND DIRECTLY BUT SENDS TO HOLOGRAM WEBSITE CLOUD WHICH REDIRECTS <<decdata>> decdata is whatever data is sent to hologram servers.
{
  if (emailRecentlySent == true) { //checking to see if the 'emailRecentlySent' flag is true. Prevents overloading emails.
    countSinceEmail = countSinceEmail + (samplingRate / 60000); //if email was recently sent, keep incrementing the 'countSinceEmail' counter with the value of samplingRate (practically, the time since the last cycle)
    if (countSinceEmail > emailFrequency) { //once countSinceEmail time is longer than emailFrequency time (both in minutes)...
      emailRecentlySent = false; //set the 'emailRecentlySent' flag to false,
      countSinceEmail = 0; //and reset the countSinceEmail counter to zero.
    }
  }

  if (emailRecentlySent == false && remainingMessages < 10) { //if (or once) an email hasn't recently been sent, connect as needed to Hologram to prepare to send the email message.
    while (!Serial);
    // Start modem and connect to Hologram's global network
    //Hologram.debug();   //starts debug to see from serial monitor if error occurs
    bool cellConnected = Hologram.begin(9600, 8888); // set baud to 9600 and start server on port 8888
    if (cellConnected) {  //if cellConnected is true, say so in serial monitor
      Serial.println(F("Cellular is connected"));
    }
    else {
      Serial.println(F("Cellular connection failed"));
    }

    for ( int count = 0; 9 - count > remainingMessages; count++) {
      Hologram.send(emailMessages[count]); //send the email with Hologram.
      Serial.print(F("Printed email Message at array value: ")); Serial.println(count);
    }


    remainingMessages = 10; //reset remainingMessages count
    emailRecentlySent = true; //set emailRecentlySent flag.

    for (int i = 0; i <= 9; i++) {
      strcpy(emailMessages[i], ""); //clear the emailMessages array.
    }

    delay(2000); //give it a little time.

    //RECONNECTION
    moduleSetup();    //since sim card can only connect to one website domain at a time, after hologram.io email connection, reconnect to original setup
  }

}

//gets keyboard input from user and puts it into variables
//FUNCTIONS FOR READING SERIAL
void flushSerial() {
  while (Serial.available())                   //flushes leftover values after every question as to not include characters from previous entries.
    Serial.read();
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) { //function to read line that user inputs per question until enter is pressed
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}


void sdCardInitialize()
{
  Serial.print(F("Initializing SD card..."));
  while (!SD.begin(53) && countSDFail < 5) {    //begin SD connection and tell user if fails more than 3 times
    Serial.println(F("initialization failed, trying again"));
    freeboardSDStatus = 1;
    delay(500);
    SD.begin(53);
    countSDFail++;
  }
  countSDFail = 0;
  if (SD.begin(53) == true)
  {
    Serial.println(F("Success in Initialization"));
    freeboardSDStatus = 0;
  }
  else
  {
    Serial.println(F("SD Initialize failed 5 times, stopping"));
    Serial.println(F("initialization done."));
  }
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  sensorStoredData = SD.open("datalog.txt", FILE_WRITE);
  if (SD.exists("datalog.txt"))
  {
    Serial.println("Success for SD");
    freeboardSDStatus = 0;
  }
  else
  {
    Serial.println("Failed");
    freeboardSDStatus = 1;
  }
}

void sdStorageVars()    //stores and formats values to be stored in SD cards with timestamps
{
  fona.getTime(timePostSD, 23);     //gets time from sim card server. (for hologram with FONA library, it can only read in the GMT time).
  sensorStoredData.print(F("Time: "));
  sensorStoredData.println(timePostSD);
  sensorStoredData.print(F("Pressure Reading: "));
  sensorStoredData.println(pressure);
  sensorStoredData.print(F("Height reading: "));
  sensorStoredData.println(distance);
  if (positionFinder() == true)   //if position moves of cursor in SD card, confirm the storage of variables, if not throw an error into the serial monitor.
  {
    Serial.println(F("Success in SD Storage"));
    freeboardSDStatus = 0;
  }
  else
  {
    Serial.println(F("Fail in SD Storage"));
    freeboardSDStatus = 1;
  }
}

void readingSD()    //This function is for reading the text in the SD card. (This is only here because my computer tower does not have a micro SD adapter.)
{
  while (sensorStoredData.available())
  {
    char letter = sensorStoredData.read();
    Serial.print(letter);
    delay(200);
  }
}

boolean positionFinder()    //function to find out if position in SD card has increased. If it increases, it means text has been stored in the card, therefore a successful storage iteration. If fails, skip and throw error.
{
  previousPositionInFile = positionInFile;
  positionInFile = sensorStoredData.position();
  if (positionInFile > previousPositionInFile)
  {
    return true;
  }
  else
  {
    return false;
  }
}
