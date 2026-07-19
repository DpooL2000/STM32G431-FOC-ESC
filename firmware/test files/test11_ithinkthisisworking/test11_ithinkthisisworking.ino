#include <SimpleFOC.h>

// --- HARDWARE CONFIG ---
const float POWER_SUPPLY = 12.0f;
const float SQRT3_2 = 0.866025f;

// Using your inline shunts
InlineCurrentSense current_sense = InlineCurrentSense(0.002f, 50.0f, PA1, PA7, PB0);

// --- MOTOR PHYSICS (A2212) ---
const float R_phase = 0.0930f;      
const float L_phase = 0.0000468f;   
const float OBS_GAIN = 5000.0f;   // Observer tuning gain

// --- STATE VARIABLES ---
float electrical_angle = 0.0;
float target_velocity = 400.0; // rad/s (Needs to be fast enough to generate BEMF)
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
  TIM1->CCER = 0xFFF; 
  TIM1->BDTR = 0x80A0; // <--- The ~940ns Dead-Time Fix (Ice cold)
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
  
  last_time = micros();
  Serial.println("Observer Tracking Test Started.");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_time) * 1e-6f;
  if(dt <= 0 || dt > 0.01f) dt = 1e-4f;
  last_time = now;

  static bool closed_loop = false;
  static int transition_counter = 0;

  // ==========================================
  // 1. ANGLE SELECTION (Direction-Aware Handoff)
  // ==========================================
  
  if (!closed_loop) {
      // 1. Open-Loop Spool Up
      electrical_angle += target_velocity * dt;

      // Handoff condition: Wait for BEMF to be strong (at least 1.5V)
      if (millis() > 3000 && (abs(E_alpha) > 1.5f || abs(E_beta) > 1.5f)) { 
          closed_loop = true;
          // IMPORTANT: On the exact moment of lock, snap the angle to the observer
          // so there is no 180-degree "jump"
          electrical_angle = obs_angle;
          Serial.println("--- SENSORLESS LOCK ENGAGED ---");
      }
  } else {
      // 2. The PLL (Phase Locked Loop)
      float angle_error = obs_angle - electrical_angle;
      
      // Shortest path wrapping (-PI to PI)
      while (angle_error > _PI) angle_error -= _2PI;
      while (angle_error < -_PI) angle_error += _2PI;

      // We pull the angle toward the observer. 
      // If the motor was spinning "backward" to the observer, 
      // this error will naturally pull the speed to match.
      electrical_angle += angle_error * 0.02f; 

      // Since we are locked, the 'target_velocity' is now just a 
      // "suggestion" to keep the integrator moving.
      electrical_angle += target_velocity * dt;
  }

  // Final Wrap
  electrical_angle = fmodf(electrical_angle, _2PI);
  if (electrical_angle < 0) electrical_angle += _2PI;

  // ==========================================
  // 2. FOC MATH (Voltage Vectors)
  // ==========================================
  float Vq = 1.5f; 
  float Vd = 0.0f;
  float s = _sin(electrical_angle), c = _cos(electrical_angle);
  
  float V_alpha = Vd * c - Vq * s;
  float V_beta  = Vd * s + Vq * c;
  
  // ==========================================
  // 3. MEASURE REAL CURRENT
  // ==========================================
  PhaseCurrent_s I = current_sense.getPhaseCurrents();
  float I_alpha = I.a;
  float I_beta = (I.a + 2.0f * I.b) * 0.57735f;

  // ==========================================
  // 4. THE OBSERVER (Direct Algebraic + Lag Comp)
  // ==========================================
  float E_alpha_raw = V_alpha - (R_phase * I_alpha);
  float E_beta_raw  = V_beta  - (R_phase * I_beta);

  float Tf = 0.005f; 
  float alpha_filter = dt / (Tf + dt);
  E_alpha = (1.0f - alpha_filter) * E_alpha + alpha_filter * E_alpha_raw;
  E_beta  = (1.0f - alpha_filter) * E_beta  + alpha_filter * E_beta_raw;

  if (abs(E_alpha) > 0.01f || abs(E_beta) > 0.01f) {
      obs_angle = _atan2(-E_alpha, E_beta);
      obs_angle += (target_velocity * Tf); // Lead compensation
      if (obs_angle < 0) obs_angle += _2PI;
      if (obs_angle > _2PI) obs_angle -= _2PI;
  }

  // ==========================================
  // 5. WRITE TO SILICON
  // ==========================================
  float Va = V_alpha, Vb = -0.5f * V_alpha + SQRT3_2 * V_beta, Vc = -0.5f * V_alpha - SQRT3_2 * V_beta;
  
  TIM1->CCR1 = (uint16_t)(_constrain((Va/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);
  TIM1->CCR2 = (uint16_t)(_constrain((Vb/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);
  TIM1->CCR3 = (uint16_t)(_constrain((Vc/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);

  //Telemetry
  static unsigned long lp = 0;
  if (millis() - lp > 1) {
    if(!closed_loop) Serial.print("SPOOLING...");
    else {
    Serial.print(obs_angle); Serial.print(","); Serial.println(electrical_angle);}
    lp = millis();
  }
}
