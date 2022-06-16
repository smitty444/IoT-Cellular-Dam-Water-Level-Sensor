 /* This is an example sketch to send battery, temperature, and GPS location data to
 *  the cloud via either HTTP GET and POST requests or via MQTT protocol. In this 
 *  sketch we will send to dweet.io, a free cloud API, as well as to ThingsBoard.io,
 *  a very powerful and free IoT platform that allows you to visualize data on dashboards.
 *  
 *  SETTINGS: You can choose to post only once or to post periodically
 *  by commenting/uncommenting line 57 ("#define samplingRate 30"). When this line is 
 *  commented out the AVR microcontroller and MCP9808 temperature sensor are put to 
 *  sleep to conserve power, but when the line is being used data will be sent to the
 *  cloud periodically. This makes it operate like a GPS tracker!
 *  
 *  PROTOCOL: You can use HTTP GET or POST requests and you can change the URL to pretty
 *  much anything you want. You can also use MQTT to publish data to different feeds
 *  on Adafruit IO. You can also subscribe to Adafruit IO feeds to command the device
 *  to do something! In order to select a protocol, simply uncomment a line in the #define
 *  section below!
 *  
 *  DWEET.IO: To check if the data was successfully sent to dweet, go to
 *  http://dweet.io/get/latest/dweet/for/{IMEI} and the IMEI number is printed at the
 *  beginning of the code but can also be found printed on the SIMCOM module itself.
 *  
 *  IoT Example Getting-Started Tutorial: https://github.com/botletics/SIM7000-LTE-Shield/wiki/GPS-Tracker-Example
 *  GPS Tracker Tutorial Part 1: https://www.instructables.com/id/Arduino-LTE-Shield-GPS-Tracking-Freeboardio/
 *  GPS Tracker Tutorial Part 2: https://www.instructables.com/id/LTE-Arduino-GPS-Tracker-IoT-Dashboard-Part-2/
 *  
 *  Author: Timothy Woo (www.botletics.com)
 *  Github: https://github.com/botletics/SIM7000-LTE-Shield
 *  Last Updated: 3/31/2021
 *  License: GNU GPL v3.0
  */

#include "Adafruit_FONA.h" // https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
#include "HologramSIMCOM.h" //library download https://github.com/HologramEducation/hologram-SIMCOM
#include <SPI.h>
#include <SD.h> 

// You don't need the following includes if you're not using MQTT
// You can find the Adafruit MQTT library here: https://github.com/adafruit/Adafruit_MQTT_Library
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

// Define *one* of the following lines:
//#define SIMCOM_2G // SIM800/808/900/908, etc.
//#define SIMCOM_3G // SIM5320
#define SIMCOM_7000
//#define SIMCOM_7070
//#define SIMCOM_7500
//#define SIMCOM_7600

// Uncomment *one* of the following protocols you want to use
// to send data to the cloud! Leave the other commented out
//#define PROTOCOL_HTTP_GET         // Generic
// #define PROTOCOL_HTTP_POST        // Generic
#define PROTOCOL_MQTT_AIO         // Adafruit IO
//#define PROTOCOL_MQTT_CLOUDMQTT   // CloudMQTT

/************************* PIN DEFINITIONS *********************************/
// For botletics SIM7000 shield
#define FONA_PWRKEY 6
#define FONA_RST 7
#define RST_PIN 45
//#define FONA_DTR 8 // Connect with solder jumper
//#define FONA_RI 9 // Need to enable via AT commands
#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX
//#define T_ALERT 12 // Connect with solder jumper

// For botletics SIM7500 shield
//#define FONA_PWRKEY 6
//#define FONA_RST 7
////#define FONA_DTR 9 // Connect with solder jumper
////#define FONA_RI 8 // Need to enable via AT commands
//#define FONA_TX 11 // Microcontroller RX
//#define FONA_RX 10 // Microcontroller TX
////#define T_ALERT 5 // Connect with solder jumper

//For HOLOGRAM connection
#define HOLO_KEY "nK>1H*)x"    //device key found on hologram dashboard website 
//char replybuffer[255]; //SSF- Unclear what this is atm????/////////////////////////////////////////////////////////////////////////////////////


// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

// Use the following line for ESP8266 instead of the line above (comment out the one above)
//SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX, false, 256); // TX, RX, inverted logic, buffer size

SoftwareSerial *fonaSerial = &fonaSS;  //enables fona serial to be displayed on serial monitor

// Hardware serial is also possible!
//HardwareSerial *fonaSerial = &Serial1;

// For ESP32 hardware serial use these lines instead
//#include <HardwareSerial.h>
//HardwareSerial fonaSS(1);

// Use this for 2G modules
#ifdef SIMCOM_2G
  Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
  
// Use this one for 3G modules
#elif defined(SIMCOM_3G)
  Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
  
// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)
// Notice how we don't include the reset pin because it's reserved for emergencies on the LTE module!
#elif defined(SIMCOM_7000) || defined(SIMCOM_7070) || defined(SIMCOM_7500) || defined(SIMCOM_7600)
  Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
#endif

#ifdef PROTOCOL_MQTT_AIO
  /************************* MQTT SETUP *********************************/
  // MQTT setup (if you're using it, that is)
  // For Adafruit IO:
  #define AIO_SERVER      "io.adafruit.com"
  #define AIO_SERVERPORT  1883
#define AIO_USERNAME    "YOUR AIO USER"
#define AIO_KEY         "YOUR AIO KEY"

  // Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
  Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
  
  // How many transmission failures in a row we're OK with before reset
  uint8_t txfailures = 0;  
  
  /****************************** MQTT FEEDS ***************************************/
  // Setup feeds for publishing.
  // Notice MQTT paths for Adafruit IO follow the form: <username>/feeds/<feedname>
  // Also notice that the combined lat/long "location" feed requires "/csv" in the name
  // The Adafruit IO map requires this format: sensor_val, lat, long, altitude
//  Adafruit_MQTT_Publish feed_location = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/location/csv"); // Group GPS data for AIO map in dashboard
//  Adafruit_MQTT_Publish feed_distance = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/distance");
//  Adafruit_MQTT_Publish feed_pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
//  Adafruit_MQTT_Publish feed_sonicSensorHt = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sonicSensorHt");
//  Adafruit_MQTT_Publish feed_maxWaterElev = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/maxWaterElev");
//  Adafruit_MQTT_Publish feed_maxWaterElevCtrl = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/maxWaterElevCtrl"); //controller feeds have to be published to be initiallized on the first loop
//  Adafruit_MQTT_Publish feed_distanceROT = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/distanceROT");
//  Adafruit_MQTT_Publish feed_publishFrequency = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/publishFrequency");
//  Adafruit_MQTT_Publish feed_publishFrequencyCtrl = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/publishFrequencyCtrl"); //controller feeds have to be published to be initiallized on the first loop
//  Adafruit_MQTT_Publish feed_pressureVolt = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressureVolt");
    Adafruit_MQTT_Publish group_WLS1_feeds = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/groups/damswls1/json");
  
  // Setup a feeds for subscribing to changes.
  ////feeds are to modify the publishing frequency and the 'maxWaterElev' alert level of the sensor.
  Adafruit_MQTT_Subscribe feedIn_publishFrequencyCtrl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/damswls1.frqc");
  Adafruit_MQTT_Subscribe feedIn_maxWaterElevCtrl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/damswls1.mhtc");

#elif defined(PROTOCOL_MQTT_CLOUDMQTT)
  /************************* MQTT SETUP *********************************/
  // For CloudMQTT find these under the "Details" tab:
  #define MQTT_SERVER      "m10.cloudmqtt.com"
  #define MQTT_SERVERPORT  16644
  #define MQTT_USERNAME    "CLOUD_MQTT_USERNAME"
  #define MQTT_KEY         "CLOUD_MQTT_KEY"
#endif

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

     /////////////NEED TO IMPROVE NAMES OF ALL 'maxWaterElev' AND 'sonicSensorHt' SUB-VARIABLES /////////////////////////////////
     ///////////// (Sorry I changed these, but lowerBound and upperBound had no meaning. -SSF)/////////////////////////
     
#define trigPin 25
#define echoPin 23 //trig and echo pin of ultrasonic sensor
long duration = 0;    //defines variables that are going to be calculated from raw output of both sensors.
float distance = 0;   //distance of ultrasonic sensor
float pressure = 0;   //pressure value read in 10 bit analog
float pressureVolt = 0; //pressure voltage reading (smoothed over 5 readings in pressureSensor() function)
float sonicSensorHt = 0; //the distance the sonic sensor believes it is from the water. (smoothed over 5 readings in ultrasonicSensor() function)
char sonicInitialValue[5] = "1234";  
char wtrHtMax[5] = "1234";
char initialWaterElev[5] = "1234";
char initialPressure[5] = "1234";    //character strings from questions
uint8_t GPScounter = 0;         //counts failed attempts at GPS location getting
float sonicInitialValueInt = 0;   //character to numerical values
float wtrHtMaxInt = 0;   //^
float depthOfWaterInt = 0;    //^^
float initialWaterElevInt = 0;   //^^^
float initialPressureInt = 0;
//char delaysBetweenPosts[5];  //number of seconds between loop function execution -> Replaced by publishFrequency, below.
float distanceROT = 0;            //distance rate of change
float pressureROT = 0;
float previousDistance = 0;       //previously measured sonic elevation value
float previousPressure = 0;       //previously measured pressure elevation value
char maxROT[8] = "0000000";
float maxROTInt = 0;
bool firstLoop  = true;   //for subscription & publishing of Adafruit IO MQTT 'Control' values.
uint16_t timeBuffer = 0;  //buffer of time, in minutes, since the last successful sensor reading was entered
uint16_t timeROT[5] = {0,0,0,0,0}; //array of ints to store the last five durations between the last five successful sensor readings
float avgUSROT[5] = {0,0,0,0,0};     //array of floats (yuck) to store the previous 5 distance-change values for ROT smoothing
float avgPROT[5] = {0,0,0,0,0};
uint8_t indexROT = 0;   //index for ROT smoothing array
float avg = 0;                 //for pressure & sonic sensor smoothing
float sample[5]; //for pressure & sonic sensor smoothing
uint8_t p = 0;                    //for pressure & sonic sensor smoothing


//email variables
uint8_t countSinceEmail = 0; //counter of time (in minutes) since the last email was sent.
char emailFrequencyInput[5] = "1234"; //defined in setup for the minimum delay between warning emails
uint8_t emailFrequency = 0; //integer of emailFrequencyInput
bool emailRecentlySent = false; //set to true when email is sent, then when countSinceEmail >= emailFrequency, set to false.
//char emailMessages[4][101] = {{""},{""},{""},{""}};
char addMessageELEV[150];
char addMessageROT[150];
//uint8_t remainingMessages = 4;
bool messageWaitingELEV =  false;
bool messageWaitingROT =  false;


//SD CARD VARIABLES
int countForReopen = 0;
char timePostSD[23];
unsigned long positionInFile = 0;
unsigned long previousPositionInFile = 0;
int freeboardSDStatus = 0;
int countSDFail = 0;
File sensorStoredData;

//JSON variables
char JSON[200];  // Make sure this is long enough for your request URL
char JSON2[200];
char CTRL[30]; // Make sure this is long enough for POST body
char latBuff[12], longBuff[12], locBuff[150], speedBuff[12],
     headBuff[12], altBuff[12], pVoltBuff[12], sonicSensorBuff[12],
     distBuff[10], pressureBuff[12], distanceROTBuff[12], sdStatusBuff[5], pressureROTBuff[12],
     publishFrequency[5], publishFrequencyCtrl[5], maxWaterElevCtrl[6]; //duration between MQTT publishing events, in MINUTES. >>--NOTE-->> samplingRate is in thousandths of a second.

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

#ifdef PROTOCOL_MQTT_AIO
  mqtt.subscribe(&feedIn_maxWaterElevCtrl); // Only if you're using MQTT
  mqtt.subscribe(&feedIn_publishFrequencyCtrl);
#endif

////Preliminary questions for the data numbers
   //Finding first height
   digitalWrite(trigPin, LOW);
   delayMicroseconds(2);
   // Trigger the sensor by setting the trigPin high for 20 microseconds. NOTE: if accuracy of measurement is an issue, try other microsecond pulses greater than 10.
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
  duration = pulseIn(echoPin, HIGH, 26000);
  sample[0] = duration * 0.034/2; //equation to change echo pin measurement to centimeters. 
  sample[0] = sample[0]*0.393700787;
  Serial.print(F("Initial Ultrasonic Sensor Reading: "));
  Serial.print(sample[0]);
  Serial.println(F(" inches"));

      flushSerial();
      Serial.println(F("Initial distance between ultrasonic sensor and water in inches (verify with reading above): "));
      readline(sonicInitialValue,4,20);
      Serial.println(sonicInitialValue);
      flushSerial();
      Serial.println(F("Initial pressure of sensor on setup in inches: "));
      readline(initialPressure,3,20);
      Serial.println(initialPressure);
      flushSerial();
      Serial.println(F("Current water surface elevation (in feet above sea level): "));
      readline(initialWaterElev,3,20);
      Serial.println(initialWaterElev);
      flushSerial();
      Serial.println(F("Maximum allowed water surface elevation (in feet above sea level): "));
      readline(wtrHtMax,3,20);
      Serial.println(wtrHtMax);
      flushSerial();
      Serial.println(F("Maximum elevation rate of change (in inch/hour): "));
      readline(maxROT,3,20);
      Serial.println(maxROT);
      flushSerial();
      Serial.println(F("How many minutes between website posts do you want? (Suggest: 3): "));
      readline(publishFrequency,3,20);
      Serial.println(publishFrequency);
      Serial.println(F("After the first email, minutes to wait before sending any additional warnings? (Suggest: 30 or 60)"));
      readline(emailFrequencyInput,3,20);
      Serial.println(emailFrequencyInput);

      emailFrequency = atoi(emailFrequencyInput);
      samplingRate = atol(publishFrequency) * 60 * 1000; // [Time in minutes] * [60 seconds per minute] * [arduino delays are in 1000ths of a second]
      
      sonicInitialValueInt = atoi(sonicInitialValue) / 12; //convert to feet, but inches are typically easier to measure and input for setup.

      strcpy(publishFrequencyCtrl,publishFrequency);
      strcpy(maxWaterElevCtrl, wtrHtMax);
      maxROTInt = atof(maxROT) * 12 / 60; //maxROT is in inch/hour for simplicity of understanding. rateOfChange() function works in foot/minute, so converting
      initialWaterElevInt = atoi(initialWaterElev);
      initialPressureInt = atoi(initialPressure) / 12; //Convert to feet, inches are easier to setup.
      wtrHtMaxInt = atoi(wtrHtMax);
//      wtrHtMaxInt = wtrHtMaxInt + initialWaterElevInt;
//      wtrHtMaxInt = wtrHtMaxInt/12;
      //Serial.println(F("*******AFTER VARIABLES IN SETUP********"));
      //postValuesToFeed();

      // initialize all the values of avgUSROT & timeROT to 0:
      for (indexROT = 0; indexROT < 4; indexROT++) {
        avgUSROT[indexROT] = 0;
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
  GPScounter=0;
  
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
  dtostrf(pressure, 1, 1, pressureBuff); // These turn the integers and floats into strings that can be read by the MQTT commands
  dtostrf(distanceROT, 1, 1, distanceROTBuff);
  //dtostrf(freeboardSDStatus, 1, 0, sdStatusBuff);
  dtostrf(pressureVolt, 1, 3, pVoltBuff);
  dtostrf(sonicSensorHt, 1, 2, sonicSensorBuff);
  dtostrf(samplingRate/60000,1,0,publishFrequency);
  dtostrf(pressureROT,1,1,pressureROTBuff);

  //postValuesToFeed(); //show values before making json

  /*
   * To fit our data into a limited buffer for sending JSON data, the feed names are reduced as follows
   * 
   *  distance = "elS" (ELevation from  ultraSonic) (at this time, distance is dictated by the ultrasonic sensor height)
   *  pressure = "elP"  (ELevation from  Pressure)
   *  sonicSensorHt = "sDat" (Sonic DATa)
   *  pressureVolt = "pDAT" (Pressure DATa)
   *  wtrHtMax = "mHt" (Maximum HeighT)
   *  distanceROT = "ROT" (rate over time)
   *  publishFreq = "frq"
   *  wtrHtMaxControl = "mHtC"
   *  publishFrequencyControl = frqC"
   *  
   *  
   */

  
  //now we make a string of json data to begin our JSON char array with the data from the above buffers
  //sprintf(JSON,"{\"feeds\":{\"elS\":\"%s\",\"elP\":\"%s\",\"sDat\":\"%s\",\"pDat\":\"%s\",\"mHt\":\"%s\",\"ROT\":\"%s\",\"frq\":\"%s\"",distBuff,pressureBuff,sonicSensorBuff,pVoltBuff,wtrHtMax,distanceROTBuff,publishFrequency);
  //sprintf(JSON,F("{\"feeds\":{\"elS\":%s,\"elP\":%s,\"sDat\":%s,\"pDat\":%s,\"mHt\":%s,\"ROT\":%s,\"frq\":%s"),distBuff,pressureBuff,sonicSensorBuff,pVoltBuff,wtrHtMax,distanceROTBuff,publishFrequency);
  sprintf(JSON,"{\"feeds\":{\"elS\":%s,\"elP\":%s,\"sDat\":%s,\"pDat\":%s}}",distBuff,pressureBuff,sonicSensorBuff,pVoltBuff);
  sprintf(JSON2,"{\"feeds\":{\"mHt\":%s,\"ROT\":%s,\"frq\":%s",wtrHtMax,distanceROTBuff,publishFrequency);
  
  // Also construct a combined, comma-separated location array (many platforms require this for dashboards, like Adafruit IO):
  //The line below is the locBuff code if the WLS is publishing data to individual FEEDS.
  sprintf(locBuff,"{\"feeds\":{\"pROT\":%s},\"location\":{\"lat\":%s,\"lon\":%s,\"ele\":%s}}",pressureROTBuff, latBuff, longBuff, altBuff); // This could look like "10,33.123456,-85.123456,120.5"

  //The line below is the locBuff code if the WLS is publishing data by passing a JSON to a GROUP of feeds. This is the ending of the JSON file, per AdafruitIO API.
  //sprintf(locBuff,"},\"location\":{\"lat\":\"%s\",\"lon\":\"%s\",\"ele\":\"%s\"}}",latBuff,longBuff,altBuff);
  //sprintf(locBuff,"},\"location\":{\"lat\":\"%s\",\"lon\":\"%s\",\"ele\":\"%s\"}}",latBuff,longBuff,altBuff);
  
  
  //Now, we combine all elements to send.
  //the 'JSON' char array we already constructed is the beginning of the JSON data.
  //We only concatenate the below 'CTRL' character array to the FEEDS portion of the JSON (the control values) on the first loop.

  if(firstLoop == true){
    sprintf(CTRL,",\"mHtC\":%s, \"frqC\":%s}}",wtrHtMax,publishFrequency);
    sprintf(JSON2,"%s%s",JSON2,CTRL);
    //strcat(JSON2,CTRL);
  }
  else{
    strcat(JSON2,"}}");
  }

  //now we concatenate the locBuff char array we created to add the location data and close out the json data.
  //strcat(JSON,locBuff);
 // strcat(JSON2,locBuff);

  //troubleshooting//
  //Serial.println(JSON);
  //Serial.println(JSON2);

/////////////////////PRINTING VALUES FOR TROUBLESHOOTING///////////////////////
    
   postValuesToFeed();
    
////////////////////////////////////////////////////////////////////////////////

#if defined(PROTOCOL_MQTT_AIO)
  // Let's use MQTT!
  
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected). See the MQTT_connect
  // function definition further below.

    MQTT_connect();
    delay(1000); //pause to let the connection commands complete.

    if(firstLoop == false){
        //////// I think that this can be done with Adafruit IO 'get' on the existing publishFrequency/wtrHtMax feeds, which will free up two feeds.
  
        // This is our 'wait for incoming subscription packets' busy subloop
        Adafruit_MQTT_Subscribe *subscription;
        while ((subscription = mqtt.readSubscription(5000))) {
          if (subscription == &feedIn_publishFrequencyCtrl) {
            Serial.print(F("*** Got feedIn_publishFrequencyCtrl Value: "));
            Serial.println((char *)feedIn_publishFrequencyCtrl.lastread);
            strcpy(publishFrequencyCtrl,feedIn_publishFrequencyCtrl.lastread); //get the new value for the control
            //Serial.print(F("*** Changing delay minutes between posts to: "));Serial.println((char *)feedIn_publishFrequencyCtrl.lastread);
            Serial.print(F("*** Changing delay minutes between posts to: "));Serial.println(publishFrequencyCtrl);
            strcpy(publishFrequency,publishFrequencyCtrl);
            samplingRate = atol(publishFrequency) * 60 * 1000; // [Time in minutes] * [60 seconds per minute] * [arduino delays are in 1000ths of a second]      
          }
          if (subscription == &feedIn_maxWaterElevCtrl) {
            Serial.print(F("*** Got feedIn_maxWaterElevCtrl Value: "));
            Serial.println((char *)feedIn_maxWaterElevCtrl.lastread);
            strcpy(maxWaterElevCtrl,feedIn_maxWaterElevCtrl.lastread);
            //Serial.print(F("*** Changing alert water level elevation to: ")); Serial.println((char *)feedIn_maxWaterElevCtrl.lastread);
            Serial.print(F("*** Changing alert water level elevation to: ")); Serial.println(maxWaterElevCtrl);
            strcpy(wtrHtMax,maxWaterElevCtrl);
            wtrHtMaxInt = atoi(wtrHtMax);
          }
        }
    }

////////////////////I'm absolutely positive that we can do the below data publishing through JSON in a group topic. see:https://io.adafruit.com/api/docs/mqtt.html?cpp#group-topics
////////////////////This would likely save a ton of data.
    
  // Now publish all the data to different feeds!
  // The MQTT_publish_checkSuccess handles repetitive stuff.
  // You can see the function near the end of this sketch.
  // For the Adafruit IO dashboard map we send the combined lat/long buffer
//    MQTT_publish_checkSuccess(feed_location, locBuff);
//    MQTT_publish_checkSuccess(feed_distance, distBuff);
//    MQTT_publish_checkSuccess(feed_pressure, pressureBuff);
//    MQTT_publish_checkSuccess(feed_publishFrequency, publishFrequency);
//    MQTT_publish_checkSuccess(feed_maxWaterElev, wtrHtMax);
//    MQTT_publish_checkSuccess(feed_distanceROT, distanceROTBuff);
//    MQTT_publish_checkSuccess(feed_pressureVolt, pVoltBuff);
//    MQTT_publish_checkSuccess(feed_sonicSensorHt, sonicSensorBuff);
//// We need to initiallize our controller feeds on the first loop
//  if(firstLoop == true){
//    MQTT_publish_checkSuccess(feed_publishFrequencyCtrl, publishFrequency);
//    MQTT_publish_checkSuccess(feed_maxWaterElevCtrl, wtrHtMax);
//      }

      delay(500);
      MQTT_publish_checkSuccess(group_WLS1_feeds, JSON); //Attempting to publish via JSON to reduce data complexity
      delay(500); //pause to let the data send.
      MQTT_publish_checkSuccess(group_WLS1_feeds, JSON2);
      delay(500); //wait for data to send
      MQTT_publish_checkSuccess(group_WLS1_feeds, locBuff);
      delay(500); //wait for data to send




  ////This is where we check to see if the controller feeds for publishFrequency and sonicSensorHt
  
//  //////// I think that this can be done with Adafruit IO 'get' on the existing publishFrequency/wtrHtMax feeds, which will free up two feeds.
//  
//  // This is our 'wait for incoming subscription packets' busy subloop
//  Adafruit_MQTT_Subscribe *subscription;
//  while ((subscription = mqtt.readSubscription(5000))) {
//    if (subscription == &feedIn_publishFrequencyCtrl) {
//      Serial.print(F("*** Got feedIn_publishFrequencyCtrl Value: "));
//      Serial.println((char *)feedIn_publishFrequencyCtrl.lastread);
//      strcpy(publishFrequencyCtrl,feedIn_publishFrequencyCtrl.lastread); //get the new value for the control
//      //Serial.print(F("*** Changing delay minutes between posts to: "));Serial.println((char *)feedIn_publishFrequencyCtrl.lastread);
//      Serial.print(F("*** Changing delay minutes between posts to: "));Serial.println(publishFrequencyCtrl);
//      strcpy(publishFrequency,publishFrequencyCtrl);
//      samplingRate = atol(publishFrequency) * 60 * 1000; // [Time in minutes] * [60 seconds per minute] * [arduino delays are in 1000ths of a second]      
//    }
//    if (subscription == &feedIn_maxWaterElevCtrl) {
//      Serial.print(F("*** Got feedIn_maxWaterElevCtrl Value: "));
//      Serial.println((char *)feedIn_maxWaterElevCtrl.lastread);
//      strcpy(maxWaterElevCtrl,feedIn_maxWaterElevCtrl.lastread);
//      //Serial.print(F("*** Changing alert water level elevation to: ")); Serial.println((char *)feedIn_maxWaterElevCtrl.lastread);
//      Serial.print(F("*** Changing alert water level elevation to: ")); Serial.println(maxWaterElevCtrl);
//      strcpy(wtrHtMax,maxWaterElevCtrl);
//      wtrHtMaxInt = atoi(wtrHtMax);
//    }
//  }
  
#endif

   sdStorageVars();       //runs variable storage and formatting to put in data into the SD card
   sensorStoredData.close();      //SD card must close after every value stored or SD card will not save the data sent to it
   Serial.println(F("Saving and Reopening SD")); 
   delay(50);
   sdCardInitialize();      //reopens SD card connection after closing
   
///////////// NOTE ////////////////
////Be aware that firstLoop influences various functions and code below this point in the loop() function may act incorrectly.
   
   sendemail();
   firstLoop=false;
   emailRecentlySent = false;
   postValuesToFeed();
   Serial.print(F("Waiting for ")); Serial.print(samplingRate/1000); Serial.println(F(" seconds\r\n"));
   delay(samplingRate); // Delay

}


void printValueSize(char *varName,char *title){
  char holding[150];
  sprintf(holding,"%s: %s",title,varName);
  Serial.println(holding);
}


void postValuesToFeed(){
    Serial.println(F("/////////////////////////////VALUES GOING TO FEED/////////////////////////////////"));
    printValueSize(pVoltBuff,"pressure voltage");
    printValueSize(sonicSensorBuff,"sonic sensor height");
    printValueSize(locBuff,"locBuff");
    printValueSize(latBuff,"latBuff");
    printValueSize(longBuff,"longBuff");
    printValueSize(distBuff,"distBuff");
    printValueSize(pressureBuff,"pressureBuff");
    printValueSize(distanceROTBuff,"distanceROTBuff");
    printValueSize(sonicInitialValue,"sonicInitialValue");
    printValueSize(wtrHtMax,"wtrHtMax");
    printValueSize(publishFrequency,"publishFrequency");
    Serial.println(F("////////////////////////TROUBLESHOOTING CONTROL VALUES/////////////////////////////"));
    Serial.print(F("samplingRate: ")); Serial.println(samplingRate);
    Serial.print(F("publishFrequency: ")); Serial.println(publishFrequency);
    Serial.print(F("publishFrequencyCtrl: ")); Serial.println(publishFrequencyCtrl);
    Serial.print(F("feedIn_publishFrequencyCtrl.lastread: ")); Serial.println((char *)feedIn_publishFrequencyCtrl.lastread);
    Serial.println(F("******"));
    Serial.print(F("wtrHtMax: ")); Serial.println(wtrHtMax);
    Serial.print(F("wtrHtMaxInt: ")); Serial.println(wtrHtMaxInt);
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
    delay(1000);
    Serial.println(F("Connecting to MQTT... "));
    GPScounter=0;
    while ((ret = mqtt.connect()) != 0 && GPScounter <= 5) { // connect will return 0 for connected
      Serial.println(mqtt.connectErrorString(ret));
      Serial.println(F("Retrying MQTT connection in 5 seconds..."));
      mqtt.disconnect();
      delay(5000);  // wait 5 seconds
      GPScounter++;
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

  if(distance>=0){ //do not discard previous distance if the existing value is zero
    previousDistance = distance;
  }
  avg = 0; //zero the value of avg.

  for(p = 0; p < 5; p++) {
    digitalWrite(trigPin, LOW);   // Clear the trigPin by setting it LOW:
    delayMicroseconds(2);
    // Trigger the sensor by setting the trigPin high for 20 microseconds. NOTE: if accuracy of measurement is an issue, try other microsecond pulses greater than 10.
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigPin, LOW);
    // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
    sample[p] = pulseIn(echoPin, HIGH);   //counts the time from when trigger pin sent the wave to when echo pin recieves the wave back
    sample[p] = sample[p] * 0.034/2;
    sample[p] = sample[p] * 0.393700787/12; //converted to centimeters (0.034/2), converted to inches (0.393700787), converted to feet (1/12)
    avg = avg + sample[p];
    delayMicroseconds(100);
  }
    //smoothed height reading = average of 5 sensor readings, 
    sonicSensorHt = avg / 5;
    distance = initialWaterElevInt - (sonicSensorHt - sonicInitialValueInt);
}



void pressureSensor()
{
    if(pressure>=0){ //do not discard previous distance if the existing value is zero
      previousPressure = pressure;
    }
    
    avg = 0; //zero the value of avg.
    
    for(p = 0; p < 5; p++){  //pressure sample smoothing (This can be improved by having an open-to-air secondary pressure sensor, and or having specific values for a sensor to calibrate zero.)
        sample[p] = analogRead(A10); //reads 10 bit value from analog 10 pin on arduino board
        avg = avg + sample[p];
        delayMicroseconds(100);
    }
    //take the average of five readings
    avg = avg/5;
    pressureVolt = avg * 5/1023;    //changes 10 bit value to voltage reading
    
    //////Experimentally derived pressure equation, linear regression of experimental data from voltage to feet of pressure head.
    if (pressureVolt < 0.39){ //if sensor reading is below .4, assume the sensor reading is in open air.
      pressure = 0;
    }
    else {
      pressure = pressureVolt*5.78282 - 2.28641;
    }    
    
    pressure = initialWaterElevInt + (pressure - initialPressureInt);  //equation to turn into above sea level

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
    float sumavgUSROT = 0;
    float sumavgPROT = 0;
    uint16_t sumTimeROT = 0;
    
    //Checks if distance or pressure meet or exceed the maximum allowed (wtrHtMaxInt). If so, send email via hologram.
    
    if ((distance >= wtrHtMaxInt || pressure >= wtrHtMaxInt) && messageWaitingELEV == false) {
      dtostrf(distance,1,2,distBuff);
      dtostrf(pressure,1,2,pressureBuff);
      sprintf(addMessageELEV,"Water elevation reading of ultrasonic sensor (%s ft) and/or pressure sensor (%s ft), exceeded: %s",distBuff,pressureBuff,wtrHtMax);
      messageWaitingELEV=true;
    }
    
    //If sonicSensorHt <= 0 OR pressureVolt =< 0.39 this is expected to mean that one of the sensors had a bad reading. So, do not add this data to the running ROT average count, but add the minute value of samplingRate to the failed time-buffer value.
    //time buffer acts to correctly show the time duration for ROT smoothing in case of a failed sensor reading.
    if ( sonicSensorHt <= 0 || pressureVolt <= 0.39) {
      timeBuffer = timeBuffer + samplingRate/60000; 
      }
    
    else {
      //set first value of avgUSROT for smoothing.
      if(previousDistance <= 0){
        avgUSROT[indexROT] = 0;
      }
      else{
        avgUSROT[indexROT] = distance - previousDistance;
      }
      //do the same thing for avgPROT for smoothing.
      if(previousPressure <= 0){
        avgPROT[indexROT] = 0;
      }
      else{
        avgPROT[indexROT] = pressure - previousPressure;
      }
      
      timeROT[indexROT] = timeBuffer + samplingRate/60000;
      //reset timeBuffer after a successful read
      timeBuffer = 0; 
      }

    //loop through and sum avgUSROT and timeROT into sumavgUSROT and sumTimeROT respectively.
    for (int sumloop = 0; sumloop < 5; sumloop++) {
      sumavgUSROT = sumavgUSROT + avgUSROT[sumloop];
      sumavgPROT = sumavgPROT + avgPROT[sumloop];
      sumTimeROT = sumTimeROT + timeROT[sumloop];
    }

    distanceROT = sumavgUSROT / sumTimeROT; //the rate over time change (rate of change) averaged per minute over the last five sensor readings & durations.
    pressureROT = sumavgPROT / sumTimeROT;
        
    if(messageWaitingROT==false){
      if ( (distanceROT > maxROTInt || -1*distanceROT > maxROTInt) || (pressureROT > maxROTInt || -1*pressureROT > maxROTInt)){ //check both, considering absolute value of rate of change, the abs()/fabs() functions seemed to be behaving unusually.
        dtostrf(distanceROT*60/12,1,1,distanceROTBuff); //convert back to in/hr for message
        dtostrf(pressureROT*60/12,1,1,pressureROTBuff);
        //sprintf(addMessageROT,"(UTC %s,%s,%s:%s)Rate of Change is: %s in/hr, exceeds: %s in/hr.",atoi(month),atoi(day),atoi(hour),atoi(minute),distanceROTBuff,maxROT);
        sprintf(addMessageROT,"Rate of Change of ultrasonic sensor (%s in/hr) or pressure sensor (%s in/hr), exceeds: %s in/hr.",distanceROTBuff,pressureROTBuff,maxROT);
        messageWaitingROT=true;
      }
    }
}


//void addEmailMessage(){ //adds the current contents of addMessage char array to the emailMessages array, based on the remainingMessages count.
//  if( remainingMessages > 0 ){
//    sprintf(emailMessages[4 - remainingMessages],"%s\r\n",addMessage); //set the lowest available message slot of the 4 available to the value of addMessage and add carriage returns and character returns
//    remainingMessages = remainingMessages - 1; //decrease the number of messages remaining counter.
//  }
//  sprintf(addMessage,""); //reset the value of addMessage to blank-ish (not completely blank).
//}

void sendemail()  //IMPORTANT: DOES NOT SEND DIRECTLY BUT SENDS TO HOLOGRAM WEBSITE CLOUD WHICH REDIRECTS <<decdata>> decdata is whatever data is sent to hologram servers.
{
  if(emailRecentlySent == true) { //checking to see if the 'emailRecentlySent' flag is true. Prevents overloading emails.
    countSinceEmail = countSinceEmail + (samplingRate/60000); //if email was recently sent, keep incrementing the 'countSinceEmail' counter with the value of samplingRate (practically, the time since the last cycle)
    if(countSinceEmail > emailFrequency){  //once countSinceEmail time is longer than emailFrequency time (both in minutes)...
      emailRecentlySent = false; //set the 'emailRecentlySent' flag to false,
      countSinceEmail = 0; //and reset the countSinceEmail counter to zero.
    }
  }
  else if( messageWaitingELEV == true || messageWaitingROT == true){
    delay(100); //waiting for printing to serial to clear
    while(!Serial);
    // Start modem and connect to Hologram's global network
    //Hologram.debug();   //starts debug to see from serial monitor if error occurs
    bool cellConnected = Hologram.begin(9600, 8888); // set baud to 9600 and start server on port 8888
    if(cellConnected) {   //if cellConnected is true, say so in serial monitor
        Serial.println(F("Cellular is connected"));
    } 
    else {
        Serial.println(F("Cellular connection failed"));
    }
      if(messageWaitingELEV == true && cellConnected == true){
        //Serial.print("Sending Message: ");Serial.println(addMessageELEV);
        Hologram.send(addMessageELEV); //send the email with Hologram.
        //Serial.println("Sent Message.");
        messageWaitingELEV=false;
        //sprintf(addMessageELEV,"0"); //clear the emailMessages array.
      }
      if(messageWaitingROT == true && cellConnected == true){
        //Serial.print("Sending Message: ");Serial.println(addMessageROT);
        Hologram.send(addMessageROT); //send the email with Hologram.  
        //Serial.println("Sent Message.");
        messageWaitingROT=false;
        //sprintf(addMessageROT,"0"); //clear the emailMessages array.
      }
             
      emailRecentlySent=true; //set emailRecentlySent flag.
            
          
    //RECONNECTION
      delay(500); //gotta give modem commands a little time.
      moduleSetup();    //since sim card can only connect to one website domain at a time, after hologram.io email connection, reconnect to original setup
  }
  
      
//  if(emailRecentlySent==false && remainingMessages < 4){ //if (or once) an email hasn't recently been sent, connect as needed to Hologram to prepare to send the email message.
//    while(!Serial);
//    // Start modem and connect to Hologram's global network
//    //Hologram.debug();   //starts debug to see from serial monitor if error occurs
//    bool cellConnected = Hologram.begin(9600, 8888); // set baud to 9600 and start server on port 8888
//    if(cellConnected) {   //if cellConnected is true, say so in serial monitor
//        Serial.println(F("Cellular is connected"));
//    } 
//    else {
//        Serial.println(F("Cellular connection failed"));
//    }
//      
//      for( int count=0; 3 - count > remainingMessages; count++){
//        Hologram.send(emailMessages[count]); //send the email with Hologram.
//        Serial.print(F("Printed email Message at array value: ")); Serial.println(count);
//      }
//      
//      
//      remainingMessages = 4; //reset remainingMessages count
//      emailRecentlySent=true; //set emailRecentlySent flag.
//
//      for(int i = 0; i <= 3; i++){
//        sprintf(emailMessages[i],""); //clear the emailMessages array.
//      }
//      
//      delay(1000); //give it a little time.
//    
//    //RECONNECTION
//      moduleSetup();    //since sim card can only connect to one website domain at a time, after hologram.io email connection, reconnect to original setup
//  }
  
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
