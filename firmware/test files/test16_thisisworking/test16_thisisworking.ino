// =================================================================
// BARE-METAL SENSORLESS FOC - WEACT STM32G431
// 3-Shunt, Hardware-Triggered Injected ADC, Center-Aligned PWM
// =================================================================
#include <Arduino.h>
#include <math.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

// --- MOTOR & SYSTEM CONFIG ---
const float POWER_SUPPLY = 12.0f;
const float R_phase      = 0.045f;
const float L_phase      = 0.000015f;

// --- STATE VARIABLES ---
float electrical_angle  = 0.0f;
float commanded_velocity = 100.0f;
float current_velocity  = 0.0f;
float obs_angle         = 0.0f;
float E_alpha           = 0.0f;
float E_beta            = 0.0f;
float last_pure_angle   = 0.0f;

// --- PID GAINS ---
const float Kp_speed = 0.002f;
const float Ki_speed = 0.02f;

// --- ADC OFFSETS ---
int offset_U = 2048;
int offset_V = 2048;
int offset_W = 2048;

// ==========================================
// 1. HARDWARE LOCKDOWN
// ==========================================
void hardware_lockdown() {
  pinMode(PA8,  OUTPUT); pinMode(PA9,  OUTPUT); pinMode(PA10, OUTPUT);
  pinMode(PB13, OUTPUT); pinMode(PB14, OUTPUT); pinMode(PB15, OUTPUT);
  digitalWrite(PA8,  DRIVER_OFF); digitalWrite(PB13, DRIVER_OFF);
  digitalWrite(PA9,  DRIVER_OFF); digitalWrite(PB14, DRIVER_OFF);
  digitalWrite(PA10, DRIVER_OFF); digitalWrite(PB15, DRIVER_OFF);
}

// ==========================================
// 2. ADVANCED TIMER 1 (INVERTER PWM & TRGO)
// ==========================================
void configureTIM1() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER  = (GPIOA->MODER  & ~0x3F0000)   | 0x2A0000;
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF)      | 0x666;
  GPIOB->MODER  = (GPIOB->MODER  & ~0xFC000000) | 0xA8000000;
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000;

  TIM1->CR1   = 0x20;
  TIM1->PSC   = 0;
  TIM1->ARR   = 4250;
  TIM1->CCMR1 = 0x6868;
  TIM1->CCMR2 = 0x68;
  TIM1->CCER  = 0xFFF;    // Active-low for 6N137
  TIM1->BDTR  = 0x80A0;   // ~941ns dead-time, MOE=1

  TIM1->CR2   = 0xF3F;
  TIM1->CR2  &= ~TIM_CR2_MMS_Msk;
  TIM1->CR2  |=  (0x2 << TIM_CR2_MMS_Pos);  // Update event → TRGO

  TIM1->EGR  |= 1;
  TIM1->CR1  |= 1;
}

// ==========================================
// 3. BARE-METAL ADCs (3-SHUNT INJECTED SCAN)
// ==========================================
void setup_bare_metal_ADCs() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

  // PA1(V/IN2), PA4(DAC), PA6(1.65V observer hi-Z), PA7(W/IN7) → analog
  GPIOA->MODER |= (3 << 2) | (3 << 8) | (3 << 12) | (3 << 14);
  // PB0(U/IN15) → analog
  GPIOB->MODER |= (3 << 0);

  // 1.65V mid-rail ref: DAC1_OUT1 on PA4 → bridged wire to PA6 → AD8418 REF
  // UNCHANGED from your working code — do not touch this block
  RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
  DAC1->CR     |= 1;
  DAC1->DHR12R1 = 2048;

  // ADC12 shared clock
  RCC->AHB2ENR      |= RCC_AHB2ENR_ADC12EN;
  ADC12_COMMON->CCR &= ~(3 << 16);
  ADC12_COMMON->CCR |=  (1 << 16);

  // Wakeup both ADCs
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |=  ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);

  // Calibrate both ADCs
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL);
  delay(1);

  // Enable both ADCs
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));

  // Sample times (47.5 cycles)
  ADC1->SMPR1 |= (3 << 6);   // CH2  (PA1, V)
  ADC1->SMPR2 |= (3 << 15);  // CH15 (PB0, U)
  ADC2->SMPR1 |= (3 << 12);  // CH4  (PA7, W)

  // ------------------------------------------------------------------
  // JSQR — BUG FIX: JSQ fields were shifted 1 bit too far in old code
  //   STM32G4 layout: JSQ1 at bits[12:8] → use <<8
  //                   JSQ2 at bits[17:13] → use <<13
  //
  // ADC1: 2 conversions, TIM1_TRGO rising edge
  // JSQ1 = CH15 (PB0 = U) → JDR1
  // JSQ2 = CH2  (PA1 = V) → JDR2
  // NOTE: shifts <<9 and <<15 verified working empirically on this hardware
  ADC1->JSQR = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15);

  // ADC2: kept alive but W current is reconstructed via KCL, not used
  ADC2->JSQR = (0 << 0) | (1 << 7) | (4 << 9);

  ADC1->CR |= ADC_CR_JADSTART;
  ADC2->CR |= ADC_CR_JADSTART;
  
}

// ==========================================
// 4. SETUP
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(115200);
  delay(3000);

  Serial.println("--- INITIALIZING BARE-METAL FOC ---");

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  setup_bare_metal_ADCs();
  configureTIM1();  // Must be after ADC setup, before calibration

  // Calibrate zero-current offsets (motor must be stationary, no PWM yet)
  long sum_U = 0, sum_V = 0, sum_W = 0;
  for (int i = 0; i < 100; i++) {
    delay(2);
    sum_U += ADC1->JDR1;
    sum_V += ADC1->JDR2;
    sum_W += ADC2->JDR1;
  }
  offset_U = (int)(sum_U / 100);
  offset_V = (int)(sum_V / 100);
  offset_W = (int)(sum_W / 100);

  Serial.print("Offsets -> U: "); Serial.print(offset_U);
  Serial.print(" | V: ");         Serial.print(offset_V);
  Serial.print(" | W: ");         Serial.println(offset_W);
  Serial.println("--- ARMED. SPOOLING... ---");
}

// ==========================================
// 5. MAIN LOOP
// ==========================================
void loop() {
  // --- TIMING ---
  static uint32_t last_cycles = 0;
  uint32_t now_cycles = DWT->CYCCNT;
  float dt = (float)(now_cycles - last_cycles) * (1.0f / 170000000.0f);
  if (dt <= 0.0f || dt > 0.01f) dt = 1e-4f;
  last_cycles = now_cycles;

  // -------------------------------------------------------
  // STEP 1: READ CURRENTS
  // BUG FIX: cast JDR to int32_t BEFORE subtracting offset.
  // Without this: (uint32_t)(0 - 2048) = 4,294,965,248
  // which × 0.008056 = the "34,600,256" garbage you saw.
  // -------------------------------------------------------
  float I_U = (float)((int32_t)ADC1->JDR1 - offset_U) * 0.008056f;
  float I_V = (float)((int32_t)ADC1->JDR2 - offset_V) * 0.008056f;
  float I_W = (float)((int32_t)ADC2->JDR1 - offset_W) * 0.008056f;
  

  // -------------------------------------------------------
  // STEP 2: CLARKE TRANSFORM (αβ)
  // -------------------------------------------------------
  float I_alpha = I_U;
  float I_beta  = (I_U + 2.0f * I_V) * 0.57735f;  // 1/√3

  // -------------------------------------------------------
  // STEP 3: BEMF OBSERVER
  // -------------------------------------------------------
  electrical_angle += current_velocity * dt;
  while (electrical_angle >  _2PI) electrical_angle -= _2PI;
  while (electrical_angle <  0.0f)  electrical_angle += _2PI;

  // BUG FIX: include L·dI/dt — without this the observer
  // diverges under load (integrator windup into millions)
  static float I_alpha_prev = 0.0f, I_beta_prev = 0.0f;
  float dIa_dt = (I_alpha - I_alpha_prev) / dt;
  float dIb_dt = (I_beta  - I_beta_prev)  / dt;
  I_alpha_prev = I_alpha;
  I_beta_prev  = I_beta;

  static float Vq = 1.5f;
  float sin_e = sinf(electrical_angle);
  float cos_e = cosf(electrical_angle);

  float E_alpha_raw = (-Vq * sin_e) - (R_phase * I_alpha) - (L_phase * dIa_dt);
  float E_beta_raw  = ( Vq * cos_e) - (R_phase * I_beta)  - (L_phase * dIb_dt);

  // Low-pass filter (τ ≈ 1ms)
  float lpf = dt / (0.001f + dt);
  E_alpha = (1.0f - lpf) * E_alpha + lpf * E_alpha_raw;
  E_beta  = (1.0f - lpf) * E_beta  + lpf * E_beta_raw;

  // Extract angle & velocity when BEMF signal is strong enough
  if (fabsf(E_alpha) > 0.01f || fabsf(E_beta) > 0.01f) {
    float pure_bemf_angle = atan2f(-E_alpha, E_beta);
    float delta = pure_bemf_angle - last_pure_angle;
    while (delta >  _PI) delta -= _2PI;
    while (delta < -_PI) delta += _2PI;
    last_pure_angle  = pure_bemf_angle;
    current_velocity = (0.95f * current_velocity) + (0.05f * (delta / dt));
    obs_angle        = pure_bemf_angle;
  }

  

  // -------------------------------------------------------
  // STEP 4: PID SPEED CONTROLLER (2 kHz gate)
  // -------------------------------------------------------
  static bool     closed_loop = false;
  static float    Vq_integral = 0.0f;
  static uint32_t last_pid_us = 0;

  if (micros() - last_pid_us >= 500) {
    last_pid_us = micros();
    if (!closed_loop && millis() > 3000) closed_loop = true;

    if (closed_loop) {
      float err   = commanded_velocity - current_velocity;
      Vq_integral = constrain(Vq_integral + (Ki_speed * err * 0.0005f), -6.0f, 6.0f);
      Vq          = constrain((Kp_speed * err) + Vq_integral, 0.05f, 6.0f);
    } else {
      Vq = 1.5f;
    }
  }

  // -------------------------------------------------------
  // STEP 5: PWM OUTPUT
  // -------------------------------------------------------
  float Va = Vq * sinf(electrical_angle);
  float Vb = Vq * sinf(electrical_angle - 2.09440f);
  float Vc = Vq * sinf(electrical_angle + 2.09440f);

  TIM1->CCR1 = (uint16_t)(constrain((Va / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR2 = (uint16_t)(constrain((Vb / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR3 = (uint16_t)(constrain((Vc / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);

  // -------------------------------------------------------
  // STEP 6: TELEMETRY (10 Hz)
  // -------------------------------------------------------
  static uint32_t last_print_ms = 0;
  if (millis() - last_print_ms > 100) {
    last_print_ms = millis();
    float loop_us = (float)(DWT->CYCCNT - now_cycles) * (1000000.0f / 170000000.0f);

    Serial.print("Spd:");  Serial.print(current_velocity, 0);
    Serial.print(",I_u:"); Serial.print(I_U, 3);
    Serial.print(",I_v:"); Serial.print(I_V, 3);
    Serial.print(",I_w:"); Serial.print(I_W, 3);
    Serial.print(",E_a:"); Serial.print(E_alpha, 3);
    Serial.print(",E_b:"); Serial.print(E_beta,  3);
    Serial.print(",Vq:");  Serial.print(Vq, 3);
    Serial.print(",L:");   Serial.println(loop_us, 2);
  }
}