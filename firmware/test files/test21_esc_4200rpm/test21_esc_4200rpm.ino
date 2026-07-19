/**
 * STM32G431 - 2200KV Ultra High Speed V/F
 * TARGET: 5,000 - 10,000 RPM
 */

#include <Arduino.h>

const int SINE_STEPS = 256;
int stepIndex = 0;
unsigned long previousMicros = 0;

// --- AGGRESSIVE RAMP ---
float currentSpeedDelay = 1000.0; 
float targetSpeedDelay = 8.0;    // Target ~4,200 RPM (Mechanical)
float acceleration = 0.2;        // Faster ramp

// --- VOLTAGE FOR HIGH SPEED ---
// We need more voltage at high speed to fight Back-EMF
float voltage_min = 900.0;  
float voltage_max = 1400.0; 

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // 1. Hardware Init (Bare Metal)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  GPIOA->MODER |= (3 << (1 * 2)) | (3 << (7 * 2)); 
  GPIOB->MODER |= (3 << (0 * 2)); 
  uint32_t op_cfg = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9);
  OPAMP1->CSR = op_cfg; OPAMP2->CSR = op_cfg; OPAMP3->CSR = op_cfg;

  // 2. Timer 1 (20kHz Center-Aligned PWM)
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
  GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 
  TIM1->PSC = 0; TIM1->ARR = 8500 - 1;    
  TIM1->CR1 |= TIM_CR1_CMS_0; 
  TIM1->CCMR1 |= (6 << 4) | (6 << 12); TIM1->CCMR2 |= (6 << 4);
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CR1 |= TIM_CR1_CEN;
}

void loop() {
  unsigned long currentMicros = micros();
  
  if (currentMicros - previousMicros >= (unsigned long)currentSpeedDelay) {
    previousMicros = currentMicros;

    // 1. RAMPING
    if (currentSpeedDelay > targetSpeedDelay) {
      currentSpeedDelay -= acceleration; 
    }

    // 2. V/F SCALING (Back-EMF Compensation)
    float speed_progress = (1000.0 - currentSpeedDelay) / (1000.0 - targetSpeedDelay);
    float v_dynamic = voltage_max - (speed_progress * (voltage_max - voltage_min));

    // 3. ANGLE MATH
    float angle = (stepIndex / (float)SINE_STEPS) * 2.0 * PI;
    float v_a = -v_dynamic * sin(angle);
    float v_b =  v_dynamic * cos(angle);

    // 4. PWM UPDATE
    TIM1->CCR1 = 4250 + (int)(v_a);
    TIM1->CCR2 = 4250 + (int)(-0.5 * v_a + 0.866 * v_b);
    TIM1->CCR3 = 4250 + (int)(-0.5 * v_a - 0.866 * v_b);

    if (++stepIndex >= SINE_STEPS) stepIndex = 0;
  }

  // Speedometer
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    float rpm = 60000000.0 / (currentSpeedDelay * SINE_STEPS * 7);
    Serial.print("RPM:"); Serial.print(rpm);
    Serial.print("  Delay:"); Serial.println(currentSpeedDelay);
  }
}