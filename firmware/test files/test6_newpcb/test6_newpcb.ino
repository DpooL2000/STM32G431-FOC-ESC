// --- Pin Mapping ---
const int pinV_Gate = PA8;  // TIM1_CH1
const int pinU_Gate = PA9;  // TIM1_CH2
const int pinW_Gate = PA10; // TIM1_CH3
const int pinSD     = PB9;  // Enable

// Internal OPAMP Non-Inverting Inputs
const int pinU_Curr = PB0;  // OPAMP3
const int pinV_Curr = PA1;  // OPAMP1
const int pinW_Curr = PA7;  // OPAMP2

void setup() {
  Serial.begin(115200);
  
  pinMode(pinU_Gate, OUTPUT);
  pinMode(pinV_Gate, OUTPUT);
  pinMode(pinW_Gate, OUTPUT);
  pinMode(pinSD, OUTPUT);

  // 1. Initial Safe States (Because of 6N137 Inversion)
  // HIGH on STM32 = LED OFF = Opto Output HIGH = Motor High-Side OFF
  digitalWrite(pinU_Gate, HIGH);
  digitalWrite(pinV_Gate, HIGH);
  digitalWrite(pinW_Gate, HIGH);

  // Enable IR2104 drivers
  digitalWrite(pinSD, HIGH); 

  // 2. Clean PWM Configuration (20kHz, 10-bit resolution)
  // 10-bit means our duty cycle range is 0 to 1023
  analogWriteFrequency(20000); 
  analogWriteResolution(10);

  // ADC Configuration (12-bit resolution: 0 to 4095)
  analogReadResolution(12);

  // --- 3. STM32G431 BARE-METAL OPAMP ACTIVATION ---
  
  // A. Enable the SYSCFG clock (Crucial for G4 OPAMPs to turn on)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // B. Configure OPAMP1 (Phase V - PA1)
  OPAMP1->CSR = 0;              // Clear all bits
  OPAMP1->CSR |= (3 << 14);     // PGGAIN: 00011 = Gain 16
  OPAMP1->CSR |= (2 << 2);      // VMSEL: 10 = PGA mode
  OPAMP1->CSR |= 1;             // OPAEN: Enable OPAMP1

  // C. Configure OPAMP2 (Phase W - PA7)
  OPAMP2->CSR = 0;
  OPAMP2->CSR |= (3 << 14); 
  OPAMP2->CSR |= (2 << 2);  
  OPAMP2->CSR |= 1;         

  // D. Configure OPAMP3 (Phase U - PB0)
  OPAMP3->CSR = 0;
  OPAMP3->CSR |= (3 << 14); 
  OPAMP3->CSR |= (2 << 2);  
  OPAMP3->CSR |= 1;         
  
  Serial.println("Bare-Metal OPAMPs Active! Checking Phase Currents...");
}

void loop() {
  // --- DRIVE PHASE U ---
  // We want 30% duty at the motor high-side.
  // Because the 6N137 inverts, STM32 must be HIGH for 70% of the time.
  // 70% of 1023 = 716
  analogWrite(pinU_Gate, 716); 
  analogWrite(pinV_Gate, 100); 
  analogWrite(pinW_Gate, 1023); 
  
  // --- READ CURRENTS ---
  float rawU = analogRead(pinU_Curr);
  float rawV = analogRead(pinV_Curr);
  float rawW = analogRead(pinW_Curr);

  // --- THE MATH ---
  // Amps = (ADC_Voltage) / (Gain * Shunt_Resistance)
  // ADC_Voltage = (Raw * 3.3V) / 4095
  // We use 0.01 because of your R010 shunts, and 16.0 for the PGA Gain
  float ampsU = (rawU * 3.3) / (4095.0 * 16.0 * 0.01);
  float ampsV = (rawV * 3.3) / (4095.0 * 16.0 * 0.01);
  float ampsW = (rawW * 3.3) / (4095.0 * 16.0 * 0.01);

  // --- PRINT RESULTS ---
  Serial.print("U: "); Serial.print(ampsU, 2);
  Serial.print("A | V: "); Serial.print(ampsV, 2);
  Serial.print("A | W: "); Serial.print(ampsW, 2);
  Serial.println("A");
  
  delay(200);
}