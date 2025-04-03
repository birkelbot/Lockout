void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("Hello world!\n");
  digitalWrite(13,HIGH);
  delay(250);
  digitalWrite(13,LOW);
  delay(250);
}
