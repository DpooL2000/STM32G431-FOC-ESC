/**
 * STM32G431 - Industrial FOC Architecture (High-Torque 1000KV Tuning)
 */

#include <Arduino.h>

const float dt = 0.0001f; 

// --- MOTOR STATE ---
volatile float electrical_angle = 0.0f;
volatile float electrical_velocity = 0.0f; 
volatile float voltage_q = 0.0f;
volatile float voltage_d = 0.0f;

// --- TUNING FOR 1000KV ---
// --- THE "CLOCK HAND" TUNING ---
float target_velocity = 10.0f;  // Extremely slow (~14 RPM)
float acceleration = 2.0f;      // Crawling acceleration (takes 5 seconds to reach max speed)
float run_voltage = 2000.0f;    // Keep this high so it has the torque to hold position
float align_voltage = 2500.0f;  // Very strong snap

enum State { ALIGNING, RAMPING };
State motor_state = ALIGNING;
unsigned long state_start_time = 0;

// --- CORDIC & FAST MATH ---
void initCORDIC() {
  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN; 
  CORDIC->CSR = (1 << 20) | (0 << 16) | (5 << 4); 
}

inline void fastSinCos(float angle_rad, float &out_sin, float &out_cos) {
  while(angle_rad > PI) angle_rad -= 2.0f * PI;
  while(angle_rad < -PI) angle_rad += 2.0f * PI;
  int32_t q31_angle = (int32_t)((angle_rad / PI) * 2147483647.0f);
  CORDIC->WDATA = q31_angle;
  out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
  out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
}

void setup() {
  Serial.begin(115200);
  initCORDIC();

  // TIM1 Setup
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
  GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 

  TIM1->CR1 = 0; 
  TIM1->PSC = 0; 
  TIM1->ARR = 8500 - 1; 
  TIM1->CR1 |= TIM_CR1_CMS_0; // Center-aligned Mode 1
  TIM1->CCMR1 |= (6 << 4) | (6 << 12); 
  TIM1->CCMR2 |= (6 << 4);
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  TIM1->BDTR |= TIM_BDTR_MOE;

  HardwareTimer *MyTim = new HardwareTimer(TIM1);
  MyTim->attachInterrupt(HighSpeed_FOC_ISR);
  TIM1->DIER |= TIM_DIER_UIE; 
  TIM1->CR1 |= TIM_CR1_CEN;

  state_start_time = millis();
}

void HighSpeed_FOC_ISR() {
  electrical_angle += electrical_velocity * dt;

  float v_sin, v_cos;
  fastSinCos(electrical_angle, v_sin, v_cos);

  float v_alpha = voltage_d * v_cos - voltage_q * v_sin;
  float v_beta  = voltage_d * v_sin + voltage_q * v_cos;

  int offset = 4250; 
  TIM1->CCR1 = offset + (int)(v_alpha);
  TIM1->CCR2 = offset + (int)(-0.5f * v_alpha + 0.866f * v_beta);
  TIM1->CCR3 = offset + (int)(-0.5f * v_alpha - 0.866f * v_beta);
}

void loop() {
  unsigned long now = millis();

  if (motor_state == ALIGNING) {
    electrical_angle = 0.0f;
    electrical_velocity = 0.0f;
    voltage_d = align_voltage; // Snap rotor to 0
    voltage_q = 0.0f;

    if (now - state_start_time > 2000) { // Increased to 2s for 1000KV inertia
      motor_state = RAMPING;
      voltage_d = 0.0f; 
      voltage_q = run_voltage; 
      state_start_time = now;
    }
  } 
  else if (motor_state == RAMPING) {
    if (electrical_velocity < target_velocity) {
      electrical_velocity += acceleration * 0.01f; 
      delay(10); 
    }
  }

  static unsigned long lastPrint = 0;
  if (now - lastPrint > 500) {
    lastPrint = now;
    Serial.print("Vq: "); Serial.print(voltage_q);
    Serial.print(" | Vel: "); Serial.println(electrical_velocity);
  }
}