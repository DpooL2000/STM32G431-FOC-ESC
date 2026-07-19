/**
 * STM32G431 - 2200KV High-Speed V/F Fast-Track
 */

#include <Arduino.h>

const int SINE_STEPS = 256;
int stepIndex = 0;
unsigned long previousMicros = 0;

// RAMP TUNING
float currentSpeedDelay = 2000.0; // Start at ~11 RPM
float targetSpeedDelay = 20.0;    // Target ~1,100 RPM (Mechanical)
float acceleration = 0.1;        // Faster ramp for 2200KV

// VOLTAGE TUNING
float voltage_q = 1200.0;  // Increased punch to ensure it doesn't stall
float voltage_d = 0.0;

volatile float Iq_filtered = 0;
volatile bool newDataReady = false;
float zeroU = 0, zeroV = 0;

// --- ADD TO GLOBALS ---
float voltage_min = 450.0;  // Voltage at high speed (Lower = Cooler)
float voltage_max = 1200.0; // Voltage to start moving
float voltage_dynamic = 1200.0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // 1. Hardware Init
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  GPIOA->MODER |= (3 << (1 * 2)) | (3 << (7 * 2)); 
  GPIOB->MODER |= (3 << (0 * 2)); 
  uint32_t op_cfg = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9);
  OPAMP1->CSR = op_cfg; OPAMP2->CSR = op_cfg; OPAMP3->CSR = op_cfg;

  // 2. Calibration
  delay(500); 
  long sU = 0, sV = 0;
  for(int i=0; i<100; i++){ sU += analogRead(PB1); sV += analogRead(PA2); delay(1); }
  zeroU = sU/100.0; zeroV = sV/100.0;

  // 3. Timer 1 (20kHz PWM)
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

    // 1. RAMPING SPEED
    if (currentSpeedDelay > targetSpeedDelay) {
      currentSpeedDelay -= acceleration; 
    }

    // 2. DYNAMIC VOLTAGE SCALING (The "Cooler")
    // This reduces the PWM power linearly as the speed increases
    float speed_progress = (2000.0 - currentSpeedDelay) / (2000.0 - targetSpeedDelay);
    voltage_dynamic = voltage_max - (speed_progress * (voltage_max - voltage_min));

    float angle = (stepIndex / (float)SINE_STEPS) * 2.0 * PI;
    
    // Inverse Park + Clarke using dynamic voltage
    float V_alpha = -voltage_dynamic * sin(angle);
    float V_beta  =  voltage_dynamic * cos(angle);

    int offset = 4250; 
    TIM1->CCR1 = offset + (int)(V_alpha);
    TIM1->CCR2 = offset + (int)(-0.5 * V_alpha + 0.866 * V_beta);
    TIM1->CCR3 = offset + (int)(-0.5 * V_alpha - 0.866 * V_beta);

    if (++stepIndex >= SINE_STEPS) stepIndex = 0;
  }

  // Telemetry
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    float rpm = 60000000.0 / (currentSpeedDelay * SINE_STEPS * 7);
    Serial.print("RPM:"); Serial.print(rpm);
    Serial.print("  V_out:"); Serial.println(voltage_dynamic);
  }
}