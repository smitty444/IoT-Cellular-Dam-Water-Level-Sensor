#include <SimpleKalmanFilter.h>
#include <NewPing.h>

#define trigPin 47
#define echoPin 46

NewPing sonar(trigPin, echoPin);

const float e_mea = 0.1;
const float e_est = 0.1;
const float q = 0.01;

SimpleKalmanFilter kf(e_mea, e_est, q);

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("*** Executing kalman_filter_tester.ino ***");
}

void loop() {
  float estimate;
  float distance = sonar.ping_in();
  if(distance > 9) {
    distance = distance/12;
    estimate = kf.updateEstimate(distance);
    Serial.print(distance); Serial.print(",");
    Serial.println(estimate);
    delay(200);
  }
  else {
    delay(50);
  }

}
