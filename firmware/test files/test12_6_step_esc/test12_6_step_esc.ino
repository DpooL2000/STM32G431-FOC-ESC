// Add these global variables at the very top of your sketch (above setup)
int step = 1;
int pwmValue = 850; // 20% Duty Cycle
int stepDelay = 4;  // Milliseconds between steps (Speed control)

// ... (Keep your existing setup() function exactly as it is) ...
void setup() {
  // 1. Enable Clocks for GPIOA and Timer 1
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  // 2. Configure Pins PA8, PA9, PA10 for Alternate Function (AF)
  // Clear the mode bits for pins 8, 9, and 10
  GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
  // Set the mode bits to '10' (Alternate Function mode)
  GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
  
  // 3. Set the Alternate Function to AF6 (Routes TIM1 to PA8, PA9, PA10 on G431)
  // First, clear the AFR bits for these pins to remove any garbage data
  GPIOA->AFR[1] &= ~((0xF << 0) | (0xF << 4) | (0xF << 8));
  // Then set them to AF6 (Value '6' is 0110 in binary)
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 

  // 4. Configure Timer 1 for 20 kHz using the 170 MHz Clock
  // Math: 170,000,000 Hz / 20,000 Hz = 8500 steps
  TIM1->PSC = 0;           // No prescaler, run at full 170 MHz
  TIM1->ARR = 8500 - 1;    // Auto-reload value (Defines the 20 kHz frequency)
  
  // 5. Configure PWM Mode 1 for Channels 1, 2, and 3
  // Clear the Output Compare Mode bits first
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
  TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M);
  // Set them to '0110' (PWM Mode 1) which is '6'
  TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
  TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos);
  
  // 6. Set Output Polarity to ACTIVE LOW
  // Because the 6N137 inverts the signal, STM32 LOW = 12V at the junction.
  // Active Low ensures that a higher CCR value physically gives a higher Junction Voltage.
  TIM1->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);

  // 7. Enable the individual Channels
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  
  // 8. Main Output Enable (MOE) - Absolutely critical for Advanced Timers like TIM1
  TIM1->BDTR |= TIM_BDTR_MOE;

  // 9. Start the Timer
  TIM1->CR1 |= TIM_CR1_CEN;
}

void loop() {
  // Execute the current step
  switch(step) {
    case 1:
      TIM1->CCR1 = pwmValue; // U High
      TIM1->CCR2 = 0;        // V Low
      TIM1->CCR3 = 0;        // W Low
      break;
    case 2:
      TIM1->CCR1 = pwmValue; // U High
      TIM1->CCR2 = pwmValue; // V High
      TIM1->CCR3 = 0;        // W Low
      break;
    case 3:
      TIM1->CCR1 = 0;        // U Low
      TIM1->CCR2 = pwmValue; // V High
      TIM1->CCR3 = 0;        // W Low
      break;
    case 4:
      TIM1->CCR1 = 0;        // U Low
      TIM1->CCR2 = pwmValue; // V High
      TIM1->CCR3 = pwmValue; // W High
      break;
    case 5:
      TIM1->CCR1 = 0;        // U Low
      TIM1->CCR2 = 0;        // V Low
      TIM1->CCR3 = pwmValue; // W High
      break;
    case 6:
      TIM1->CCR1 = pwmValue; // U High
      TIM1->CCR2 = 0;        // V Low
      TIM1->CCR3 = pwmValue; // W High
      break;
  }

  // Move to the next step
  step++;
  if(step > 6) {
    step = 1; // Reset back to step 1 to complete the circle
  }

  // Wait before taking the next step (This controls the RPM)
  delay(stepDelay); 
}