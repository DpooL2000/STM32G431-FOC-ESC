// Pins
const int PWM_A = PA8;
const int PWM_B = PA10;
const int SD_PIN = PB8;
const int LED_ONBOARD = PC6; // Usually PA5 or PC13 on STM32

void setup() {
  pinMode(SD_PIN, OUTPUT);
  pinMode(LED_ONBOARD, OUTPUT);
  
  digitalWrite(SD_PIN, HIGH); // Enable the IR2104 drivers

  // Set high frequency PWM (20kHz)
  analogWriteFrequency(20000); 
  
  // Set resolution to 10-bit (0-1023)
  analogWriteResolution(10);
}

void loop() {
  // --- FORWARD PHASE ---
  digitalWrite(LED_ONBOARD, HIGH); // Onboard LED HIGH for Forward
  
  // Phase A: 10% PWM (Inverted for 6N137: 1023 - 102 = 921)
  analogWrite(PWM_A, 921); 
  
  // Phase B: Must be fully OFF to Ground the other side of the motor
  // 100% High on STM32 Pin = 0V out of 6N137 = IR2104 Low-Side ON (GND)
  analogWrite(PWM_B, 1023); 
  
  delay(5000);

  // --- REVERSE PHASE ---
  digitalWrite(LED_ONBOARD, LOW); // Onboard LED LOW for Reverse
  
  // Phase A: Set to 100% High (Stops PWM, keeps IR2104 Low-Side ON)
  analogWrite(PWM_A, 1023); 
  
  // Phase B: 10% PWM
  analogWrite(PWM_B, 921); 
  
  delay(5000);
}