// --- SPWM Open-Loop Variables ---
const int SINE_STEPS = 256;
int sineTable[SINE_STEPS];
int stepIndex = 0;
unsigned long previousMicros = 0;
int speedDelay = 500; // 3ms per step (Slow, safe, won't stall)

// --- FOC Measurement Variables ---
volatile float I_alpha = 0.0;
volatile float I_beta  = 0.0;
volatile float I_d     = 0.0;
volatile float I_q     = 0.0;
volatile bool newDataReady = false;

// Hardware Calibration Variables
float zeroU = 0;
float zeroV = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // --- 1. GENERATE THE SAFE SINE TABLE ---
  int safePWM = 1300; // ~15% power to break cogging torque smoothly
  int amplitude = safePWM / 2;
  int offset = safePWM / 2; 
  
  for(int i = 0; i < SINE_STEPS; i++) {
    float angle = (i / (float)SINE_STEPS) * 2.0 * PI;
    sineTable[i] = offset + (int)(amplitude * sin(angle));
  }

  // --- 2. BARE METAL OPAMP SETUP ---
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

  GPIOA->MODER |= (3 << (1 * 2)) | (3 << (7 * 2)); 
  GPIOB->MODER |= (3 << (0 * 2)); 

  // 16x Gain (Shifted by 5 for STM32G4)
  OPAMP1->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);
  OPAMP2->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);
  OPAMP3->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);

  // --- 3. AUTO-CALIBRATION ROUTINE ---
  // Wait for analog signals to stabilize
  delay(500); 
  long sumU = 0;
  long sumV = 0;
  
  // Read 100 times to find the exact hardware bias (the 47k + 1k divider)
  for(int i = 0; i < 100; i++) {
    sumU += analogRead(PB1);
    sumV += analogRead(PA2);
    delay(2);
  }
  
  zeroU = sumU / 100.0;
  zeroV = sumV / 100.0;
  
  // Print calibration values just so you know what they are
  Serial.print("Calibrated Zero U: "); Serial.println(zeroU);
  Serial.print("Calibrated Zero V: "); Serial.println(zeroV);
  delay(1000); // Give you a second to read it before motor starts

  // --- 4. BARE METAL TIMER 1 SETUP (20kHz) ---
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
  GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
  
  GPIOA->AFR[1] &= ~((0xF << 0) | (0xF << 4) | (0xF << 8));
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 

  TIM1->PSC = 0;           
  TIM1->ARR = 8500 - 1;    
  
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
  TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M);
  TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
  TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos);
  
  TIM1->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  TIM1->BDTR |= TIM_BDTR_MOE;

  // --- 5. HARDWARE INTERRUPT (ADC SYNC) ---
  HardwareTimer *MyTim = new HardwareTimer(TIM1);
  MyTim->attachInterrupt(syncMeasurementISR);
  TIM1->DIER |= TIM_DIER_UIE; 

  TIM1->CR1 |= TIM_CR1_CEN;
}

// --- THE INTERRUPT SERVICE ROUTINE (ISR) ---
void syncMeasurementISR() {
  static int isr_divider = 0;

  // Run at 100Hz to keep the main loop perfectly smooth
  if (++isr_divider >= 200) { 
    isr_divider = 0; 

    int rawU = analogRead(PB1); 
    int rawV = analogRead(PA2); 

    // Subtract the calibrated Zero Point!
    float ampsU = (((rawU - zeroU) / 4095.0) * 3.3) / 0.16;
    float ampsV = (((rawV - zeroV) / 4095.0) * 3.3) / 0.16;
    
    // 1. CLARKE TRANSFORM
    I_alpha = ampsU;
    I_beta = 0.57735 * (ampsU + 2.0 * ampsV); 

    // 2. PARK TRANSFORM (Rotates to the rotor angle)
    float angle = (stepIndex / (float)SINE_STEPS) * 2.0 * PI;
    float cosA = cos(angle);
    float sinA = sin(angle);

    I_d = I_alpha * cosA + I_beta * sinA;   // Flux (Wasted Heat)
    I_q = -I_alpha * sinA + I_beta * cosA;  // Torque (Actual turning force)
    
    newDataReady = true;
  }
}

void loop() {
  // 1. Plot the FOC Heartbeat
  if (newDataReady) {
    newDataReady = false; 
    
    // We are now plotting the final DC transformed values
    Serial.print("Id_Flux:"); Serial.print(I_d, 3); Serial.print(",");
    Serial.print("Iq_Torque:"); Serial.println(I_q, 3);
  }

  // 2. Output the 3-Phase AC Sine Waves (NON-BLOCKING)
  unsigned long currentMicros = micros();
  
  if (currentMicros - previousMicros >= speedDelay) {
    previousMicros = currentMicros;

    TIM1->CCR1 = sineTable[stepIndex];
    TIM1->CCR2 = sineTable[(stepIndex + 85) % SINE_STEPS];
    TIM1->CCR3 = sineTable[(stepIndex + 170) % SINE_STEPS];

    stepIndex++;
    if(stepIndex >= SINE_STEPS) stepIndex = 0;
  }
}