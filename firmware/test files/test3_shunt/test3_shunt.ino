// Pins
const int PWM_A = PA8;
const int PWM_B = PA10;
const int SD_PIN = PB8;
const int LED_ONBOARD = PC6;

void setup() {
  Serial.begin(115200);
  
  pinMode(SD_PIN, OUTPUT);
  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(SD_PIN, HIGH); 

  // PWM Configuration (20kHz, 10-bit)
  analogWriteFrequency(20000); 
  analogWriteResolution(10);

  // ADC Configuration (12-bit for precision)
  analogReadResolution(12);

  // --- STM32G431 INTERNAL OPAMP ACTIVATION ---
  // Enable OPAMP1 (for PB0/A0) with Gain 8
  OPAMP1->CSR |= (0x07 << 11); // PGA Gain to 8x
  OPAMP1->CSR |= 0x01;         // Enable OPAMP1
  
  // Enable OPAMP2 (for PA7/A7) with Gain 8
  OPAMP2->CSR |= (0x07 << 11); // PGA Gain to 8x
  OPAMP2->CSR |= 0x01;         // Enable OPAMP2
}

void loop() {
  // FORWARD: Phase A Active
  digitalWrite(LED_ONBOARD, HIGH);
  analogWrite(PWM_A, 820);  // ~20% Duty (1023 - 204 = 819)
  analogWrite(PWM_B, 1023); // Phase B grounded
  
  // Read current while the low-side is switching
  // Math: (ADC_Value * 3.3V / 4096) / (Gain 8 * 0.1 Ohm Shunt)
  float rawA = analogRead(PA7);
  float ampsA = (rawA * 3.3) / (4096.0 * 8.0 * 0.17);

  Serial.print("Forward Amps: ");
  Serial.println(ampsA, 3);
  
  delay(2000);

  // REVERSE: Phase B Active
  digitalWrite(LED_ONBOARD, LOW);
  analogWrite(PWM_A, 1023); // Phase A grounded
  analogWrite(PWM_B, 820);  // ~20% Duty
  
  float rawB = analogRead(PB0);
  float ampsB = (rawB * 3.3) / (4096.0 * 8.0 * 0.17);

  Serial.print("Reverse Amps: ");
  Serial.println(ampsB, 3);

  delay(2000);
}