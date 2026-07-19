#include <SimpleFOC.h>

// --- HARDWARE CONFIG ---
const float POWER_SUPPLY = 12.0f;
const float SQRT3_2 = 0.866025f;

// Using your inline shunts
InlineCurrentSense current_sense = InlineCurrentSense(0.002f, 50.0f, PA1, PA7, PB0);

// --- MOTOR PHYSICS (A2212) ---
const float R_phase = 0.045f;      
const float L_phase = 0.000015f;   
const float OBS_GAIN = 5000.0f;   // Observer tuning gain

// --- STATE & CONTROL VARIABLES ---
float electrical_angle = 0.0;
float commanded_velocity = 5000.0; // The speed you ACTUALLY want (rad/s)
float pll_velocity = 400.0;        // Open-loop spool up speed
float current_velocity = 0.0;      // Filtered real-time speed

unsigned long last_time = 0;

// --- OBSERVER VARIABLES ---
float I_alpha_est = 0, I_beta_est = 0;
float E_alpha = 0, E_beta = 0;
float obs_angle = 0;

void configureTIM1() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER = (GPIOA->MODER & ~0x3F0000) | 0x2A0000; 
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF) | 0x666;
  GPIOB->MODER = (GPIOB->MODER & ~0xFC000000) | 0xA8000000; 
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000; 
  TIM1->CR1 = 0x20; TIM1->PSC = 0; TIM1->ARR = 4250;
  TIM1->CCMR1 = 0x6868; TIM1->CCMR2 = 0x68;
  
  // CRITICAL SAFETY: 6N137 Active-Low Inversion
  TIM1->CCER = 0xFFF; 
  
  // CRITICAL SAFETY: ~940ns Dead-Time Fix (Ice cold)
  TIM1->BDTR = 0x80A0; 
  
  TIM1->CR2 = 0xF3F; 
  TIM1->EGR |= 1; TIM1->CR1 |= 1; 
}

void setup() {
  Serial.begin(2000000);
  analogReadResolution(12);
  
  // DAC for 1.65V Reference
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC1EN);
  GPIOA->MODER |= (3 << 8) | (3 << 12); 
  DAC1->CR |= 1; DAC1->DHR12R1 = 2048;

  configureTIM1();
  
  // Calibrate current offsets using the fresh 1.65V reference
  current_sense.init(); 

  // --- UNLOCK DWT CYCLE COUNTER ---
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable Trace Debug
  DWT->CYCCNT = 0;                                // Reset the counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable Cycle Counter
  
  last_time = micros();
  Serial.println("Observer Tracking Test Started.");
}

void loop() {
  // 1. Capture exact start cycle (1 CPU clock cycle)
  uint32_t start_cycles = DWT->CYCCNT; 
  
  // --- ULTRA-FAST dt CALCULATION ---
  static uint32_t last_cycles = 0;
  // 170MHz clock = 1 / 170,000,000 = 0.00000000588 seconds per cycle
  float dt = (float)(start_cycles - last_cycles) * 0.00000000588235f; 
  if(dt <= 0.0f || dt > 0.01f) dt = 1e-4f;
  last_cycles = start_cycles;

  static bool closed_loop = false;
  float Vq = 0.0f; 
  float Vd = 0.0f;

  // ==========================================
  // 1. SPOOL UP & HANDOFF
  // ==========================================
  if (!closed_loop) {
      electrical_angle += pll_velocity * dt;
      Vq = 1.5f; 

      // Replaced abs() with fabsf() for hardware floating point speed
      if (millis() > 3000 && (fabsf(E_alpha) > 1.5f || fabsf(E_beta) > 1.5f)) { 
          closed_loop = true;
          electrical_angle = obs_angle;
          Serial.println("--- SENSORLESS LOCK ENGAGED ---");
      }
  } else {
      // --- CLOSED-LOOP PID TRACKING ---
      static float last_obs_angle = obs_angle;
      float delta_angle = obs_angle - last_obs_angle;
      
      while (delta_angle > _PI) delta_angle -= _2PI;
      while (delta_angle < -_PI) delta_angle += _2PI;
      last_obs_angle = obs_angle;
      
      float raw_vel = delta_angle / dt;
      float alpha_vel = dt / (0.01f + dt);
      current_velocity = (1.0f - alpha_vel) * current_velocity + alpha_vel * raw_vel;

      float Kp = 0.015f; 
      float Ki = 0.15f;  
      float max_Vq = 6.0f; 
      
      float vel_error = commanded_velocity - current_velocity;
      static float Vq_integral = 0.0f;
      
      Vq_integral += vel_error * Ki * dt;
      Vq_integral = _constrain(Vq_integral, -max_Vq, max_Vq); 
      
      Vq = (Kp * vel_error) + Vq_integral;
      Vq = _constrain(Vq, 0.4f, max_Vq); 

      // --- PLL ---
      float angle_error = obs_angle - electrical_angle;
      while (angle_error > _PI) angle_error -= _2PI;
      while (angle_error < -_PI) angle_error += _2PI;

      electrical_angle += angle_error * 0.02f; 
      electrical_angle += current_velocity * dt; 
  }

  // --- FAST WRAP (Replaces the extremely slow fmodf function) ---
  while (electrical_angle >= _2PI) electrical_angle -= _2PI;
  while (electrical_angle < 0.0f) electrical_angle += _2PI;

  // ==========================================
  // 2. FOC MATH & ADC READ
  // ==========================================
  float s = _sin(electrical_angle), c = _cos(electrical_angle);
  float V_alpha = Vd * c - Vq * s;
  float V_beta  = Vd * s + Vq * c;
  
  // NOTE: If your loop is still slow, this exact line below is the culprit!
  PhaseCurrent_s I = current_sense.getPhaseCurrents();
  float I_alpha = I.a;
  float I_beta = (I.a + 2.0f * I.b) * 0.57735f;

  // ==========================================
  // 3. THE OBSERVER
  // ==========================================
  float E_alpha_raw = V_alpha - (R_phase * I_alpha);
  float E_beta_raw  = V_beta  - (R_phase * I_beta);

  float Tf = 0.005f; 
  float alpha_filter = dt / (Tf + dt);
  E_alpha = (1.0f - alpha_filter) * E_alpha + alpha_filter * E_alpha_raw;
  E_beta  = (1.0f - alpha_filter) * E_beta  + alpha_filter * E_beta_raw;

  if (fabsf(E_alpha) > 0.01f || fabsf(E_beta) > 0.01f) {
      obs_angle = _atan2(-E_alpha, E_beta);
      obs_angle += (current_velocity * Tf); 
      while (obs_angle < 0.0f) obs_angle += _2PI;
      while (obs_angle >= _2PI) obs_angle -= _2PI;
  }

  // ==========================================
  // 4. WRITE TO SILICON
  // ==========================================
  float Va = V_alpha, Vb = -0.5f * V_alpha + SQRT3_2 * V_beta, Vc = -0.5f * V_alpha - SQRT3_2 * V_beta;
  
  TIM1->CCR1 = (uint16_t)(_constrain((Va/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);
  TIM1->CCR2 = (uint16_t)(_constrain((Vb/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);
  TIM1->CCR3 = (uint16_t)(_constrain((Vc/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);

  // ==========================================
  // 5. CYCLE TIMING TELEMETRY 
  // ==========================================
  uint32_t end_cycles = DWT->CYCCNT;
  uint32_t cycles_taken = end_cycles - start_cycles;

  static unsigned long lp = 0;
  if (millis() - lp > 100) {
      float loop_time_us = (float)cycles_taken * 0.00588235f; 
      Serial.print("Cycles: "); Serial.print(cycles_taken);
      Serial.print(" | Loop: "); Serial.print(loop_time_us, 2); Serial.println(" us");
      lp = millis();
  }
}