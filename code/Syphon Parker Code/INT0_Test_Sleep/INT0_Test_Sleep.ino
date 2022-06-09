//Parker Lovett
//Created: 5/26/2022 Last Edited: 6/1/2022
//Test to wake from sleep
//WORKS PIN 2 INT0 ON ARDUINO MEGA 2560

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>

 
int counter = 0;
int n = counter;
double contactValTime = 0;
int counterTimes = 0;
int contactVal = 0;
int prevContactVal = 0;
void setup() {
  Serial.begin(9600);
  //Gets initial sensor val
  contactVal = digitalRead(2);
  prevContactVal = contactVal;
  //configure interrupt, MEGA can only do LOW interrupts to wake
  //attachInterrupt(digitalPinToInterrupt(2), ISR__Wake, LOW);
  Serial.println("Success Setup");
}
void loop() {
  delay(1000);
  counter++;
  contactVal = digitalRead(2);
  Serial.print("Awake for ");
  Serial.print(counter, DEC);
  Serial.println(" second");
  Serial.print("Contact Value: ");
  Serial.println(contactVal);
  ///sensorReadDelay();
  if(counter == 3)
  {
    Serial.println("Entering sleep");
    delay(200);
    counter = 0;
    enterSleep();
  }
}
void ISR__Wake() {
  sleep_disable();
  delay(100);
  Serial.println("Success ISR call");
  contactVal = digitalRead(2);
  Serial.print("Contact Sensor Value ");
  Serial.println(contactVal);
  detachInterrupt(0);
}

void enterSleep(void)
{
  
  /* Setup pin2 as an interrupt and attach handler. */
  attachInterrupt(digitalPinToInterrupt(2), ISR__Wake, LOW);
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  sleep_enable();
  
  sleep_mode();
  
  /* The program will continue from here. */
  /* First thing to do is disable sleep. */
  contactVal = digitalRead(2);
  Serial.println("Woken up success ");
  Serial.print("Contact Sensor Value ");
  Serial.println(contactVal);
  sleep_disable(); 
  contactVal = digitalRead(2);
  Serial.println("Sleep Pin Disabled");
}

void sensorReadDelay()
{
  if (prevContactVal == contactVal)
  {
    delay(1000);
  }
  prevContactVal = contactVal;
}
