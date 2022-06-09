int micVol = 0;
int micToggle = 0;
void setup(){
  pinMode(A5, INPUT);
  pinMode(47, INPUT);
  Serial.begin(115200);
}
void loop(){
  micVol = analogRead(A5);
  micToggle = digitalRead(47);
  Serial.print("Mic Reading: ");
  Serial.print(micVol);
  Serial.print(   "Mic Toggle: ");
  Serial.print(micToggle);
  Serial.println("");
  delay(150);
}
