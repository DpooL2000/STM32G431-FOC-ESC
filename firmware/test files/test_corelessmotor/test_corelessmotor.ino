const int PWM_A = PA8;
const int PWM_B = PA10;
const int SD_PIN = PB8;
const int LED_ONBOARD = PC6;

// 3.7V / 12V * 1023 = 315 (Safety Limit)
const int MAX_SAFE_DUTY = 315; 

void setup() {
  Serial.begin(115200);
  pinMode(SD_PIN, OUTPUT);
  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(SD_PIN, HIGH);

  analogWriteFrequency(20000); 
  analogWriteResolution(10);
  analogReadResolution(12);

  // Enable Internal OPAMPs for Shunts
  OPAMP1->CSR |= (0x07 << 11) | 0x01; // PB0, Gain 8
  OPAMP2->CSR |= (0x07 << 11) | 0x01; // PA7, Gain 8
}

void loop() {
  // --- Direction A -> B ---
  digitalWrite(LED_ONBOARD, HIGH);
  // 6N137 Inversion: (1023 - duty) for PWM, 1023 for Ground
  analogWrite(PWM_A, 1023 - MAX_SAFE_DUTY); 
  analogWrite(PWM_B, 1023); 
  
  printCurrents();
  delay(2000);

  // --- Stop / Brake ---
  digitalWrite(LED_ONBOARD, LOW);
  analogWrite(PWM_A, 1023);
  analogWrite(PWM_B, 1023);
  delay(1000);

  // --- Direction B -> A ---
  analogWrite(PWM_A, 1023); 
  analogWrite(PWM_B, 1023 - MAX_SAFE_DUTY); 
  
  printCurrents();
  delay(2000);

  // --- Stop / Brake ---
  analogWrite(PWM_A, 1023);
  analogWrite(PWM_B, 1023);
  delay(1000);
}

void printCurrents() {
  float ampA = (analogRead(PB0) * 3.3) / (4096.0 * 8.0 * 0.1);
  float ampB = (analogRead(PA7) * 3.3) / (4096.0 * 8.0 * 0.1);

  Serial.print(ampA, 3);
  Serial.print(" ");
  Serial.println(ampB, 3);
}