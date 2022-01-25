#include <NewPing.h>

#define trigPin 3
#define echoPin 4

NewPing sonar(trigPin, echoPin);
float initial_distance = 0;
float sea_level = 1000;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("*** Executing ping_tester.ino ***");
  
  while(initial_distance <= 0) {
    initial_distance = sonar.ping_in();
    delay(50);
  }
  initial_distance = initial_distance/12;
  Serial.print("initial distance: "); Serial.print(initial_distance); Serial.println(" ft");
  delay(50);
}

void loop() {
  float distance = sonar.ping_in();
  if(distance > 0) {
    distance = distance/12;
    Serial.print("distance: "); Serial.print(distance); Serial.println(" ft");
    distance = sea_level + (initial_distance - distance);
    Serial.print("above sea level: "); Serial.print(distance); Serial.println(" ft");
    delay(1000);
  }
  else {
    delay(50);
  }
}
