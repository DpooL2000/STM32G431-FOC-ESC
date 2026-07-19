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

// --- STATE VARIABLES ---
float electrical_angle = 0.0;
float target_velocity = 15.0; // rad/s (Needs to be fast enough to generate BEMF)
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
  Serial.begin(115200);
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

  // ==========================================
  // 1. OPEN-LOOP VOLTAGE COMMAND
  // ==========================================
  float Vq = 1.5f; // Pushing a safe 1.5V
  float Vd = 0.0f;
  
  electrical_angle += target_velocity * dt;
  if (electrical_angle > _2PI) electrical_angle -= _2PI;

  float s = _sin(electrical_angle), c = _cos(electrical_angle);
  
  // Calculate Commanded Voltages
  float V_alpha = Vd * c - Vq * s;
  float V_beta  = Vd * s + Vq * c;
  
  // ==========================================
  // 2. MEASURE REAL CURRENT
  // ==========================================
  PhaseCurrent_s I = current_sense.getPhaseCurrents();
  float I_alpha = I.a;
  float I_beta = (I.a + 2.0f * I.b) * 0.57735f;

  // ==========================================
  // 3. THE OBSERVER (Direct Algebraic Method)
  // ==========================================
  // 1. Calculate raw BEMF using pure Ohms Law (E = V - IR)
  float E_alpha_raw = V_alpha - (R_phase * I_alpha);
  float E_beta_raw  = V_beta  - (R_phase * I_beta);

  // 2. Heavy Low-Pass Filter to kill the PWM and ADC noise
  // A time constant (Tf) of ~5ms gives a beautiful smooth wave
  float Tf = 0.005f; 
  float alpha_filter = dt / (Tf + dt);
  
  E_alpha = (1.0f - alpha_filter) * E_alpha + alpha_filter * E_alpha_raw;
  E_beta  = (1.0f - alpha_filter) * E_beta  + alpha_filter * E_beta_raw;

  // 3. Extract Angle
  if (abs(E_alpha) < 0.001f && abs(E_beta) < 0.001f) {
      obs_angle = electrical_angle; // Follow open-loop at dead stop
  } else {
      obs_angle = _atan2(-E_alpha, E_beta);
      obs_angle -= _PI_2;
      
      // Wrap to 0-2PI
      if (obs_angle < 0) obs_angle += _2PI;
      if (obs_angle > _2PI) obs_angle -= _2PI;
  }

  // ==========================================
  // 4. WRITE TO SILICON
  // ==========================================
  float Va = V_alpha, Vb = -0.5f * V_alpha + SQRT3_2 * V_beta, Vc = -0.5f * V_alpha - SQRT3_2 * V_beta;
  
  TIM1->CCR1 = (uint16_t)(_constrain((Va/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);
  TIM1->CCR2 = (uint16_t)(_constrain((Vb/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);
  TIM1->CCR3 = (uint16_t)(_constrain((Vc/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);

  // ==========================================
  // 5. TELEMETRY
  // ==========================================
  static unsigned long lp = 0;
  if (millis() - lp > 20) {
    // We expect these two sawtooth waves to lock together
    Serial.print("OpenLoop_Angle:"); Serial.print(electrical_angle); Serial.print(",");
    Serial.print("Observer_Angle:"); Serial.println(obs_angle);

    // Serial.println("\n\n====== SYSTEM CRASH DUMP ======");
    // Serial.print("dt (Seconds): "); Serial.println(dt, 6);
    // Serial.print("V_alpha: "); Serial.println(V_alpha, 4);
    // Serial.print("V_beta: "); Serial.println(V_beta, 4);
    // Serial.print("REAL I_alpha: "); Serial.println(I_alpha, 4);
    // Serial.print("REAL I_beta: "); Serial.println(I_beta, 4);
    // Serial.println("-------------------------------");
    // Serial.print("dt_L (dt / L): "); Serial.println(dt_L, 4);
    // Serial.print("denom: "); Serial.println(denom, 4);
    // Serial.print("EST I_alpha: "); Serial.println(I_alpha_est, 4);
    // Serial.print("err_I_alpha: "); Serial.println(err_I_alpha, 4);
    // Serial.print("EST E_alpha: "); Serial.println(E_alpha, 4);
    // Serial.print("obs_angle: "); Serial.println(obs_angle, 4);
    // Serial.println("===============================\n");

    lp = millis();
  }
}