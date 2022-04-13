#include <NewPing.h>

#define trigPin 47
#define echoPin 46

NewPing sonar(trigPin, echoPin);
float initial_distance = 0;
float distance = 0;

#define WINDOW_SIZE 5

int INDEX = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;
int c = 1;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("*** Executing ping_tester.ino ***");

//  while (initial_distance <= 0) {
//    initial_distance = sonar.ping_in();
//    delay(50);
//  }
//  initial_distance = initial_distance / 12;
//  Serial.print("initial distance: "); Serial.print(initial_distance); Serial.println(" ft");
//  delay(50);
}

void loop() {
  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  while(c) {
    distance = sonar.ping_in();
    if (distance > 9) {
      distance = distance / 12;
      c = 0;
      delay(200);
    }
    else {
      delay(50);
    }
  }

  READINGS[INDEX] = distance;           // Add the newest reading to the window
  SUM = SUM + distance;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result

  Serial.print(distance);
  Serial.print(",");
//  Serial.print(SUM);
//  Serial.print(",");
  Serial.println(AVERAGED);
  c=1;
  delay(300); 
}
