const int PWM_A = PA8;
const int PWM_B = PA10;
const int SD_A = PB8;
const int SD_B = PB9;
const int SENSE_A = PA7;
const int SENSE_B = PB0;

const int MAX_TIMER = 255;
const int DUTY_30 = 144; 

unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(20000); 
  
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(SD_A, OUTPUT);
  pinMode(SD_B, OUTPUT);

  digitalWrite(SD_A, HIGH); 
  digitalWrite(SD_B, HIGH);
  analogWrite(PWM_A, MAX_TIMER); 
  analogWrite(PWM_B, MAX_TIMER); 

  OPAMP1->CSR |= (0x07 << 11);
  OPAMP1->CSR |= 0x01;
  OPAMP2->CSR |= (0x07 << 11);
  OPAMP2->CSR |= 0x01;

  Serial.println("Send 'f', 'r', 's'");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'f') {
      digitalWrite(SD_A, LOW);  
      digitalWrite(SD_B, LOW);  
      analogWrite(PWM_B, MAX_TIMER); 
      analogWrite(PWM_A, MAX_TIMER - DUTY_30); 
      Serial.println("Forward 20kHz 8-bit");
    } 
    else if (cmd == 'r') {
      digitalWrite(SD_A, LOW); 
      digitalWrite(SD_B, LOW);
      analogWrite(PWM_A, MAX_TIMER); 
      analogWrite(PWM_B, MAX_TIMER - DUTY_30); 
      Serial.println("Reverse 20kHz 8-bit");
    } 
    else if (cmd == 's') {
      digitalWrite(SD_A, HIGH); 
      digitalWrite(SD_B, HIGH);
      analogWrite(PWM_A, MAX_TIMER); 
      analogWrite(PWM_B, MAX_TIMER);
      Serial.println("Stop");
    }
  }

  if (millis() - lastPrintTime > 500) {
    float rawA = analogRead(SENSE_A);
    float rawB = analogRead(SENSE_B);
    
    float ampsA = (rawA * 3.3) / (4096.0 * 8.0 * 0.2);
    float ampsB = (rawB * 3.3) / (4096.0 * 8.0 * 0.2);

    Serial.print("Amps A: "); Serial.print(ampsA, 3);
    Serial.print(" | Amps B: "); Serial.println(ampsB, 3);
    
    lastPrintTime = millis();
  }
}