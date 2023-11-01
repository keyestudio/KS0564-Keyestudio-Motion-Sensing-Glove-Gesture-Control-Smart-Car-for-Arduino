void setup() {
  Serial.begin(115200);// Initialize the serial port, and set the baud rate to 115200
}

void loop() {
  Serial.println("A");
  delay(500);
  Serial.println("B");
  delay(500);
}
