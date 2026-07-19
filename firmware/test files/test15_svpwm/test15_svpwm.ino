// --- SPWM Variables ---
const int SINE_STEPS = 256;
int sineTable[SINE_STEPS];

int stepIndex = 0;
int speedDelay = 50; // Microseconds between sine steps (Controls RPM)

void setup() {
  // --- 1. GENERATE THE SINE TABLE ---
  // We center the sine wave at 850 (10% duty) with an amplitude of 850.
  // This means the duty cycle sweeps smoothly from 0 to 1700 (20% Max).
  int amplitude = 850;
  int offset = 850; 
  
  for(int i = 0; i < SINE_STEPS; i++) {
    // Calculate the angle in radians for this step
    float angle = (i / (float)SINE_STEPS) * 2.0 * PI;
    // Calculate the PWM value and store it
    sineTable[i] = offset + (int)(amplitude * sin(angle));
  }

  // --- 2. BARE METAL TIMER 1 SETUP (20kHz) ---
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
  GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
  
  GPIOA->AFR[1] &= ~((0xF << 0) | (0xF << 4) | (0xF << 8));
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 

  TIM1->PSC = 0;           
  TIM1->ARR = 8500 - 1;    // 170MHz / 20kHz
  
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
  TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M);
  TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
  TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos);
  
  // Active Low Polarity to handle your 6N137 Optocoupler inversions
  TIM1->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  TIM1->BDTR |= TIM_BDTR_MOE;

  TIM1->CR1 |= TIM_CR1_CEN;
}

void loop() {
  // --- 3. APPLY 3-PHASE AC SINE WAVES ---
  // Phase U: Current step
  TIM1->CCR1 = sineTable[stepIndex];
  
  // Phase V: Offset by 120 degrees (256 steps / 3 = 85)
  TIM1->CCR2 = sineTable[(stepIndex + 85) % SINE_STEPS];
  
  // Phase W: Offset by 240 degrees (85 * 2 = 170)
  TIM1->CCR3 = sineTable[(stepIndex + 170) % SINE_STEPS];

  // Move to the next micro-step in the sine wave
  stepIndex++;
  if(stepIndex >= SINE_STEPS) {
    stepIndex = 0; 
  }

  // Delay to control the rotational speed
  // 2000us * 256 steps = 0.5 seconds per electrical revolution
  delayMicroseconds(speedDelay); 
}