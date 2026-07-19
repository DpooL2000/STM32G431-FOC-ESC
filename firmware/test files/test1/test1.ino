void setup() {
  Serial.begin(115200);
  pinMode(PC13, OUTPUT);
  pinMode(PA5,  OUTPUT);
  pinMode(PB8,  OUTPUT);
  pinMode(PC6,  OUTPUT);
}

void loop() {
  
  // Test PC6
  Serial.println("Testing PC6...");
  digitalWrite(PC6, LOW); delay(500); digitalWrite(PC6, HIGH); delay(500);
}