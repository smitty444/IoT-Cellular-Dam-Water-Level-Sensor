const int LED = 13;
const int sensorPin = A10;

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(sensorPin, INPUT);

}

void loop() {
  int state = digitalRead(sensorPin);
  Serial.print("State: "); Serial.println(state);
  if(state == HIGH) {
    digitalWrite(LED, HIGH);
  }
  else {
    digitalWrite(LED, LOW);
  }
  delay(500);
}
