int counter = 0;
int n = counter;
double contactValTime = 0;
void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), ISR__Wake, CHANGE);
  Serial.println("Success Setup");
}
void loop() {
  if (n != counter)
  {
    Serial.println(counter);
    n = counter;
  }
}
void ISR__Wake() {
  if (millis() > contactValTime+100) {
    counter++;
    contactValTime = millis();
    Serial.println("Interrupt Func Success");
  }
}
