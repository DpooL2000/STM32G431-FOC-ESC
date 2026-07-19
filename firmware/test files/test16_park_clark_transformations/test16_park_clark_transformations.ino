// --- SPWM Open-Loop Variables ---
const int SINE_STEPS = 256;
int sineTable[SINE_STEPS];
int stepIndex = 0;
int speedDelay = 500; // 500us = ~67 RPM

// --- FOC Measurement Variables ---
volatile float I_alpha = 0.0;
volatile float I_beta = 0.0;
volatile bool newDataReady = false;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // --- 1. GENERATE THE SAFE SINE TABLE ---
  // Max duty is 8% (680) to protect the shunts from burning at low speeds
  int safePWM = 1100; 
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

  // --- 3. BARE METAL TIMER 1 SETUP (20kHz) ---
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

  // --- 4. HARDWARE INTERRUPT (ADC SYNC) ---
  HardwareTimer *MyTim = new HardwareTimer(TIM1);
  MyTim->attachInterrupt(syncMeasurementISR);
  TIM1->DIER |= TIM_DIER_UIE; 

  TIM1->CR1 |= TIM_CR1_CEN;
}

// --- THE INTERRUPT SERVICE ROUTINE (ISR) ---
void syncMeasurementISR() {
  static int isr_divider = 0;
  isr_divider++;

  // Read current at 100Hz to keep CPU free
  if (isr_divider >= 200) { 
    isr_divider = 0; 

    // Read Phase U and V (We only need two for Clarke Transform!)
    int rawU = analogRead(PB1); 
    int rawV = analogRead(PA2); 

    float ampsU = ((rawU / 4095.0) * 3.3) / 0.16;
    float ampsV = ((rawV / 4095.0) * 3.3) / 0.16;
    
    // --- THE CLARKE TRANSFORM ---
    // Mathematically translates 3-phase AC into a 2D stationary grid
    I_alpha = ampsU;
    I_beta = 0.57735 * (ampsU + 2.0 * ampsV); 
    
    newDataReady = true;
  }
}

void loop() {
  // 1. Plot the FOC Heartbeat
  if (newDataReady) {
    newDataReady = false; 
    
    // Print formatted for the Arduino Serial Plotter
    Serial.print("Alpha:"); Serial.print(I_alpha, 3); Serial.print(",");
    Serial.print("Beta:"); Serial.println(I_beta, 3);
  }

  // 2. Output the 3-Phase AC Sine Waves (Open-Loop)
  TIM1->CCR1 = sineTable[stepIndex];
  TIM1->CCR2 = sineTable[(stepIndex + 85) % SINE_STEPS];
  TIM1->CCR3 = sineTable[(stepIndex + 170) % SINE_STEPS];

  stepIndex++;
  if(stepIndex >= SINE_STEPS) stepIndex = 0;

  delayMicroseconds(speedDelay); 
}