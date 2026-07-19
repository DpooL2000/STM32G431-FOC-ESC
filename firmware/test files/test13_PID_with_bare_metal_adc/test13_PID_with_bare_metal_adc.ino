// =================================================================
// BARE-METAL SENSORLESS FOC - WEACT STM32G431 
// The 11-Microsecond Loop
// =================================================================
#include <Arduino.h>
#include <math.h>

#define _PI 3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

// --- MOTOR & SYSTEM CONFIG ---
const float POWER_SUPPLY = 12.0f;
const float SQRT3_2 = 0.866025f;
const float R_phase = 0.045f;      
const float L_phase = 0.000015f;   

// --- STATE & CONTROL VARIABLES ---
float electrical_angle = 0.0;
float commanded_velocity = 800.0; // Target Speed (rad/s)
float pll_velocity = 400.0;        // Open-loop spool up speed
float current_velocity = 0.0;      
float obs_angle = 0;
float E_alpha = 0, E_beta = 0;

// --- ADC OFFSETS ---
int offset_U = 2048;
int offset_W = 2048;

// ==========================================
// 1. HARDWARE LOCKDOWN (Anti-Shoot-Through)
// ==========================================
void hardware_lockdown() {
  pinMode(PA8,  OUTPUT); pinMode(PA9,  OUTPUT); pinMode(PA10, OUTPUT);
  pinMode(PB13, OUTPUT); pinMode(PB14, OUTPUT); pinMode(PB15, OUTPUT);

  // Force 6N137 optocouplers to safe HIGH state instantly
  digitalWrite(PA8, DRIVER_OFF);  digitalWrite(PB13, DRIVER_OFF); 
  digitalWrite(PA9, DRIVER_OFF);  digitalWrite(PB14, DRIVER_OFF); 
  digitalWrite(PA10, DRIVER_OFF); digitalWrite(PB15, DRIVER_OFF); 
}

// ==========================================
// 2. ADVANCED TIMER 1 (INVERTER PWM)
// ==========================================
void configureTIM1() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER = (GPIOA->MODER & ~0x3F0000) | 0x2A0000; 
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF) | 0x666;
  GPIOB->MODER = (GPIOB->MODER & ~0xFC000000) | 0xA8000000; 
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000; 
  
  TIM1->CR1 = 0x20; TIM1->PSC = 0; TIM1->ARR = 4250;
  TIM1->CCMR1 = 0x6868; TIM1->CCMR2 = 0x68;
  TIM1->CCER = 0xFFF;   // 6N137 Active-Low Inversion
  TIM1->BDTR = 0x80A0;  // 940ns Dead-Time
  TIM1->CR2 = 0xF3F; 
  TIM1->EGR |= 1; TIM1->CR1 |= 1; 
}

// ==========================================
// 3. BARE-METAL ADC & DAC ARCHITECTURE
// ==========================================
void setup_bare_metal_ADCs() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  
  // High-Z Safety Lock for PA1, PA4 (DAC), PA6 (Probe), PA7, PB0
  GPIOA->MODER |= (3 << 2) | (3 << 8) | (3 << 12) | (3 << 14); 
  GPIOB->MODER |= (3 << 0);             
  
  // 1.65V DAC Reference
  RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;       
  DAC1->CR |= 1;                            
  DAC1->DHR12R1 = 2048;                     

  // ADC Wakeup & Calibration
  RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
  ADC12_COMMON->CCR &= ~(3 << 16); ADC12_COMMON->CCR |= (1 << 16);
  
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);
  
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL); delay(1);

  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));

  ADC1->SMPR1 |= (3 << 6); ADC1->SMPR2 |= (3 << 15); ADC2->SMPR1 |= (3 << 12);
}

// FAST READ: Phase U (PB0 -> ADC1, Ch 15)
uint16_t fast_read_Phase_U() {
  ADC1->SQR1 = (15 << 6); ADC1->CR |= ADC_CR_ADSTART;        
  while(!(ADC1->ISR & ADC_ISR_EOC)); return ADC1->DR;                   
}

// FAST READ: Phase W (PA7 -> ADC2, Ch 4)
uint16_t fast_read_Phase_W() {
  ADC2->SQR1 = (4 << 6); ADC2->CR |= ADC_CR_ADSTART;        
  while(!(ADC2->ISR & ADC_ISR_EOC)); return ADC2->DR;                   
}

// ==========================================
// 4. MAIN SETUP
// ==========================================
void setup() {
  hardware_lockdown(); // Secure the gates instantly

  Serial.begin(115200);
  delay(3000); // Wait for USB CDC to mount
  
  Serial.println("--- INITIALIZING BARE-METAL FOC ---");
  
  // Unlock Hardware Cycle Counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
  DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            

  setup_bare_metal_ADCs();
  
  long sum_U = 0, sum_W = 0;
  for(int i = 0; i < 100; i++) {
      sum_U += fast_read_Phase_U();
      sum_W += fast_read_Phase_W();
      delay(1);
  }
  offset_U = sum_U / 100;
  offset_W = sum_W / 100;
  
  Serial.print("Offsets Calibrated -> V(PA1): "); Serial.print(offset_U);
  Serial.print(" | W(PA7): "); Serial.println(offset_W);
  
  // Finally, hand the pins over to TIM1 hardware
  configureTIM1();
  Serial.println("--- INVERTER ARMED & READY. SPOOLING... ---");
}

// ==========================================
// 5. THE 11-MICROSECOND MAIN LOOP
// ==========================================
void loop() {
  uint32_t start_cycles = DWT->CYCCNT; 
  
  // --- A. FAST TIMING ---
  static uint32_t last_cycles = 0;
  float dt = (float)(start_cycles - last_cycles) * 0.00000000588235f; 
  if(dt <= 0.0f || dt > 0.01f) dt = 1e-4f;
  last_cycles = start_cycles;

  static bool closed_loop = false;
  float Vq = 0.0f, Vd = 0.0f;

  // --- B. READ CURRENT & PHASE ALIGNMENT ---
  int raw_U = fast_read_Phase_U(); // Phase A
  int raw_W = fast_read_Phase_W(); // Phase C
  
  float I_a = (float)(raw_U - offset_U) * 0.008056f;
  float I_c = (float)(raw_W - offset_W) * 0.008056f;
  
  // Kirchhoff's Law: Deduce Phase B
  float I_b = -I_a - I_c; 

  // Clarke Transform (Now perfectly aligned with physical coils)
  float I_alpha = I_a;
  float I_beta = (I_a + 2.0f * I_b) * 0.57735f;

  // --- C. SPOOL UP & HANDOFF ---
  if (!closed_loop) {
      electrical_angle += pll_velocity * dt;
      Vq = 1.5f; // Open-loop push
      
      if (millis() > 3000 && (fabsf(E_alpha) > 0.3f || fabsf(E_beta) > 0.3f)) {
          closed_loop = true;
          electrical_angle = obs_angle;
          Serial.println("--- SENSORLESS LOCK ENGAGED ---");
      }
  } else {
      // --- D. CLOSED-LOOP PID TRACKING ---
      static float last_obs_angle = obs_angle;
      float delta_angle = obs_angle - last_obs_angle;
      while (delta_angle > _PI) delta_angle -= _2PI;
      while (delta_angle < -_PI) delta_angle += _2PI;
      last_obs_angle = obs_angle;
      
      float raw_vel = delta_angle / dt;
      float alpha_vel = dt / (0.01f + dt);
      current_velocity = (1.0f - alpha_vel) * current_velocity + alpha_vel * raw_vel;

      float vel_error = commanded_velocity - current_velocity;
      static float Vq_integral = 0.0f;
      Vq_integral += vel_error * 0.15f * dt; // Ki = 0.15
      
      // Hardware accelerated float constraints
      Vq_integral = fmaxf(-3.0f, fminf(Vq_integral, 3.0f)); 
      Vq = (0.015f * vel_error) + Vq_integral; 
      Vq = fmaxf(0.4f, fminf(Vq, 3.0f)); // Hard cap at 3V

      // PLL Angle Update
      float angle_error = obs_angle - electrical_angle;
      while (angle_error > _PI) angle_error -= _2PI;
      while (angle_error < -_PI) angle_error += _2PI;
      electrical_angle += angle_error * 0.02f + (current_velocity * dt); 
  }

  while (electrical_angle >= _2PI) electrical_angle -= _2PI;
  while (electrical_angle < 0.0f) electrical_angle += _2PI;

  // --- E. FOC MATH & BEMF OBSERVER ---
  float s = sinf(electrical_angle), c = cosf(electrical_angle);
  float V_alpha = Vd * c - Vq * s;
  float V_beta  = Vd * s + Vq * c;
  
  float Tf = 0.005f; 
  float alpha_filter = dt / (Tf + dt);
  E_alpha = (1.0f - alpha_filter) * E_alpha + alpha_filter * (V_alpha - (R_phase * I_alpha));
  E_beta  = (1.0f - alpha_filter) * E_beta  + alpha_filter * (V_beta  - (R_phase * I_beta));

  if (fabsf(E_alpha) > 0.01f || fabsf(E_beta) > 0.01f) {
      obs_angle = atan2f(-E_alpha, E_beta);
      obs_angle += (current_velocity * Tf); 
      while (obs_angle < 0.0f) obs_angle += _2PI;
      while (obs_angle >= _2PI) obs_angle -= _2PI;
  }

  // --- F. WRITE TO SILICON ---
  float Va = V_alpha, Vb = -0.5f * V_alpha + SQRT3_2 * V_beta, Vc = -0.5f * V_alpha - SQRT3_2 * V_beta;
  
  float d_a = fmaxf(0.05f, fminf((Va/POWER_SUPPLY)+0.5f, 0.95f));
  float d_b = fmaxf(0.05f, fminf((Vb/POWER_SUPPLY)+0.5f, 0.95f));
  float d_c = fmaxf(0.05f, fminf((Vc/POWER_SUPPLY)+0.5f, 0.95f));

  TIM1->CCR1 = (uint16_t)(d_a * 4250);
  TIM1->CCR2 = (uint16_t)(d_b * 4250);
  TIM1->CCR3 = (uint16_t)(d_c * 4250);

  // --- G. TELEMETRY ---
  uint32_t cycles_taken = DWT->CYCCNT - start_cycles;
  static unsigned long lp = 0;
  if (millis() - lp > 200) {
      float loop_time_us = (float)cycles_taken * 0.00588235f; 
      Serial.print("Speed: "); Serial.print(current_velocity, 0);
      Serial.print(" rad/s | Vq: "); Serial.print(Vq, 2);
      Serial.print("V | Loop Time: "); Serial.print(loop_time_us, 2); Serial.println(" us");
      lp = millis();
  }
}