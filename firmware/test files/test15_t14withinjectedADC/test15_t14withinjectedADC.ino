// =================================================================
// BARE-METAL SENSORLESS FOC - WEACT STM32G431 
// The 11-Microsecond Loop (Hardware-Triggered Injected Scan: U & V)
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
float commanded_velocity = 100.0; // Target Speed (rad/s)
float pll_velocity = 50.0;        // Open-loop spool up speed
float current_velocity = 0.0;      
float obs_angle = 0;
float E_alpha = 0, E_beta = 0;

// --- PID CONTROLLER GAINS ---
float Kp_speed = 0.002f;  // Drastically reduced (was 0.015)
float Ki_speed = 0.02f;   // Drastically reduced (was 0.15)

// --- ADC OFFSETS ---
int offset_U = 2048;
int offset_V = 2048;

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
// 2. ADVANCED TIMER 1 (INVERTER PWM & TRGO)
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
  // Force TIM1 to generate a TRGO pulse on Update Event (Center of PWM)
  TIM1->CR2 &= ~TIM_CR2_MMS_Msk; 
  TIM1->CR2 |= (0x2 << TIM_CR2_MMS_Pos); 
  
  TIM1->EGR |= 1; TIM1->CR1 |= 1; 
}

// ==========================================
// 3. BARE-METAL INJECTED ADC (U & V ONLY)
// ==========================================
void setup_bare_metal_ADCs() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  
  // High-Z Safety Lock for PA1 (V), PA4 (DAC), PA7 (W-Ignored), PB0 (U)
  GPIOA->MODER |= (3 << 2) | (3 << 8) | (3 << 14); 
  GPIOB->MODER |= (3 << 0);             
  
  // 1.65V DAC Reference
  RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;       
  DAC1->CR |= 1;                            
  DAC1->DHR12R1 = 2048;                     

  // ADC Wakeup & Calibration (Only ADC1 Needed)
  RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
  ADC12_COMMON->CCR &= ~(3 << 16); ADC12_COMMON->CCR |= (1 << 16);
  
  ADC1->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; delay(1);
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));

  ADC1->SMPR1 |= (3 << 6); ADC1->SMPR2 |= (3 << 15);

  // INJECTED SEQUENCE SETUP
  // ADC1 JSQR: Length=2 (U then V), Trigger=Rising Edge, Source=TIM1_TRGO (0)
  // Channel 15 (PB0) -> JDR1
  // Channel 2 (PA1)  -> JDR2
  ADC1->JSQR = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15);
  
  // ARM THE TRIGGER!
  ADC1->CR |= ADC_CR_JADSTART;
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
  
  // CRITICAL: Start TIM1 before calibration so hardware triggers the ADCs!
  configureTIM1();
  
  long sum_U = 0, sum_V = 0;
  for(int i = 0; i < 100; i++) {
      delay(2); // Wait for hardware triggers to populate JDRs in background
      sum_U += ADC1->JDR1;
      sum_V += ADC1->JDR2;
  }
  offset_U = sum_U / 100;
  offset_V = sum_V / 100;
  
  Serial.print("Offsets Calibrated -> U(PB0): "); Serial.print(offset_U);
  Serial.print(" | V(PA1): "); Serial.println(offset_V);
  Serial.println("--- INVERTER ARMED & READY. SPOOLING... ---");
}

// ==========================================
// 5. THE MAIN FOC LOOP
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

  // --- B. READ BACKGROUND CURRENT & CLARKE TRANSFORM ---
  // Zero software latency. The silicon already grabbed these perfectly centered.
  int raw_U = ADC1->JDR1; // Phase A
  int raw_V = ADC1->JDR2; // Phase B
  
  float I_a = (float)(raw_U - offset_U) * 0.008056f;
  float I_b = (float)(raw_V - offset_V) * 0.008056f;

  // Clarke Transform (Updated mathematically to use exactly A and B)
  float I_alpha = I_a;
  float I_beta = (I_a + 2.0f * I_b) * 0.57735f;

  // --- C. SPOOL UP & HANDOFF ---
  if (!closed_loop) {
      electrical_angle += pll_velocity * dt;
      Vq = 1.5f; // Safe Open-loop push
      
      if (millis() > 3000 && (fabsf(E_alpha) > 0.3f || fabsf(E_beta) > 0.3f)) {
          closed_loop = true;
          electrical_angle = obs_angle;
          Serial.println("--- SENSORLESS LOCK ENGAGED ---");
      }
  } 
  // else {
  //     // --- D. CLOSED-LOOP PID TRACKING ---
  //     static float last_obs_angle = obs_angle;
  //     float delta_angle = obs_angle - last_obs_angle;
  //     while (delta_angle > _PI) delta_angle -= _2PI;
  //     while (delta_angle < -_PI) delta_angle += _2PI;
  //     last_obs_angle = obs_angle;
      
  //     float raw_vel = delta_angle / dt;
  //     float alpha_vel = dt / (0.01f + dt);
  //     current_velocity = (1.0f - alpha_vel) * current_velocity + alpha_vel * raw_vel;

  //     float vel_error = commanded_velocity - current_velocity;
  //     static float Vq_integral = 0.0f;
  //     Vq_integral += vel_error * 0.15f * dt; // Ki = 0.15
      
  //     // Hardware accelerated float constraints
  //     Vq_integral = fmaxf(-3.0f, fminf(Vq_integral, 3.0f)); 
  //     Vq = (0.015f * vel_error) + Vq_integral; 
  //     Vq = fmaxf(0.4f, fminf(Vq, 3.0f)); // Hard cap at 3V

  //     // PLL Angle Update
  //     float angle_error = obs_angle - electrical_angle;
  //     while (angle_error > _PI) angle_error -= _2PI;
  //     while (angle_error < -_PI) angle_error += _2PI;
  //     electrical_angle += angle_error * 0.02f + (current_velocity * dt); 
  // }
  else {
      // --- D. CLOSED-LOOP PID TRACKING ---
      static float last_obs_angle = obs_angle;
      float delta_angle = obs_angle - last_obs_angle;
      while (delta_angle > _PI) delta_angle -= _2PI;
      while (delta_angle < -_PI) delta_angle += _2PI;
      last_obs_angle = obs_angle;
      
      // Low-pass filter the speed to prevent PID violence
      float raw_vel = delta_angle / dt;
      float alpha_vel = dt / (0.05f + dt); // Heavier filter (0.05s time constant)
      current_velocity = (1.0f - alpha_vel) * current_velocity + alpha_vel * raw_vel;

      // The actual PID Controller
      float vel_error = commanded_velocity - current_velocity;
      static float Vq_integral = 0.0f;
      
      Vq_integral += vel_error * Ki_speed * dt; 
      
      // Anti-windup (Limit integral to +/- 2.0V)
      Vq_integral = fmaxf(-2.0f, fminf(Vq_integral, 2.0f)); 
      
      Vq = (Kp_speed * vel_error) + Vq_integral; 
      
      // Hard cap total voltage at 2.0V for safety during tuning
      Vq = fmaxf(0.4f, fminf(Vq, 2.0f)); 

      // Update electrical angle to track the observer
      float angle_error = obs_angle - electrical_angle;
      while (angle_error > _PI) angle_error -= _2PI;
      while (angle_error < -_PI) angle_error += _2PI;
      electrical_angle += angle_error * 0.1f + (current_velocity * dt); 
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
  // float Va = V_alpha, Vb = -0.5f * V_alpha + SQRT3_2 * V_beta, Vc = -0.5f * V_alpha - SQRT3_2 * V_beta;
  // NEW CORRECT MATH: Vb is lagging (-)
  float Va = V_alpha;
  float Vb = -0.5f * V_alpha - SQRT3_2 * V_beta; 
  float Vc = -0.5f * V_alpha + SQRT3_2 * V_beta;
  
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