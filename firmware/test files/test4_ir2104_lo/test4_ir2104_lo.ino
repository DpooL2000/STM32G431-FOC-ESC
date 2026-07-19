const int PWM_A = PA8;
const int PWM_B = PA10;
const int SD_A = PB8;
const int SD_B = PB9;

void setup() {
  Serial.begin(115200);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(SD_A, OUTPUT);
  pinMode(SD_B, OUTPUT);

  // START STATE: Shutdown Both (5V) and PWM High (IN = 0V)
  digitalWrite(SD_A, HIGH); 
  digitalWrite(SD_B, HIGH);
  digitalWrite(PWM_A, HIGH); 
  digitalWrite(PWM_B, HIGH); 

  Serial.println("--- Inverted Logic Dual Bridge Diagnostic ---");
  Serial.println("A: '1'=Enable, '2'=Shutdown, '3'=IN High (HO), '4'=IN Low (LO)");
  Serial.println("B: '5'=Enable, '6'=Shutdown, '7'=IN High (HO), '8'=IN Low (LO)");
  Serial.println("'0'=SHUTDOWN ALL");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // --- BRIDGE A ---
    if (cmd == '1') { digitalWrite(SD_A, LOW);  Serial.println("Bridge A ENABLED (0V)"); }
    else if (cmd == '2') { digitalWrite(SD_A, HIGH); Serial.println("Bridge A SHUTDOWN (5V)"); }
    else if (cmd == '3') { digitalWrite(PWM_A, LOW);  Serial.println("Bridge A IN=5V (HO Test)"); }
    else if (cmd == '4') { digitalWrite(PWM_A, HIGH); Serial.println("Bridge A IN=0V (LO Test)"); }
    
    // --- BRIDGE B ---
    else if (cmd == '5') { digitalWrite(SD_B, LOW);  Serial.println("Bridge B ENABLED (0V)"); }
    else if (cmd == '6') { digitalWrite(SD_B, HIGH); Serial.println("Bridge B SHUTDOWN (5V)"); }
    else if (cmd == '7') { digitalWrite(PWM_B, LOW);  Serial.println("Bridge B IN=5V (HO Test)"); }
    else if (cmd == '8') { digitalWrite(PWM_B, HIGH); Serial.println("Bridge B IN=0V (LO Test)"); }
    
    // --- EMERGENCY STOP ---
    else if (cmd == '0') {
      digitalWrite(SD_A, HIGH);
      digitalWrite(SD_B, HIGH);
      Serial.println("ALL SHUTDOWN");
    }
  }
}