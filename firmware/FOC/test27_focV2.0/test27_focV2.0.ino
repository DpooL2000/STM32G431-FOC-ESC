// =================================================================
// BARE-METAL SENSORLESS FOC - WEACT STM32G431 (V2.2 RACER KERNEL)
// Park-and-Spin State Machine | Observer Fix | CORDIC | SVPWM
// =================================================================
#include <Arduino.h>
#include <math.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

HardwareSerial ELRS_Serial(PB7, PB6); 
const int THROTTLE_CH = 2; // Channel 3 (AETR)

// --- MOTOR & SYSTEM CONFIG ---
const float POWER_SUPPLY = 12.0f;
const float R_phase      = 0.045f;
const float L_phase      = 0.000015f;

// --- STATE VARIABLES ---
float elrs_throttle      = 0.0f; 
bool  is_armed           = false;

float electrical_angle   = 0.0f;
float current_velocity   = 0.0f;
float obs_angle          = 0.0f;
float E_alpha            = 0.0f;
float E_beta             = 0.0f;
float last_pure_angle    = 0.0f;

// --- STARTUP STATE MACHINE ---
enum FOCState { IDLE, ALIGN, RAMP, CLOSED_LOOP };
FOCState motor_state = IDLE;
uint32_t state_timer = 0;

// To feed the observer accurate data
float V_alpha_cmd = 0.0f;
float V_beta_cmd  = 0.0f;

// --- ADC OFFSETS ---
int offset_U = 2048;
int offset_V = 2048;
int offset_W = 2048;

// ==========================================
// 1. HARDWARE CORDIC INTERFACE
// ==========================================
inline void get_Fast_Sin_Cos(float angle_rad, float* out_sin, float* out_cos) {
  while(angle_rad > _PI) angle_rad -= 2.0f * _PI;
  while(angle_rad < -_PI) angle_rad += 2.0f * _PI;
  int32_t angle_q31 = (int32_t)((angle_rad / _PI) * 2147483648.0f);
  CORDIC->WDATA = angle_q31;
  *out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
  *out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
}

// ==========================================
// 2. HARDWARE LOCKDOWN
// ==========================================
void hardware_lockdown() {
  pinMode(PA8,  OUTPUT); pinMode(PA9,  OUTPUT); pinMode(PA10, OUTPUT);
  pinMode(PB13, OUTPUT); pinMode(PB14, OUTPUT); pinMode(PB15, OUTPUT);
  digitalWrite(PA8,  DRIVER_OFF); digitalWrite(PB13, DRIVER_OFF);
  digitalWrite(PA9,  DRIVER_OFF); digitalWrite(PB14, DRIVER_OFF);
  digitalWrite(PA10, DRIVER_OFF); digitalWrite(PB15, DRIVER_OFF);
}

// ==========================================
// 3. ADVANCED TIMER 1 (INVERTER PWM & TRGO)
// ==========================================
void configureTIM1() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER  = (GPIOA->MODER  & ~0x3F0000)   | 0x2A0000;
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF)      | 0x666;
  GPIOB->MODER  = (GPIOB->MODER  & ~0xFC000000) | 0xA8000000;
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000;

  TIM1->CR1   = 0x20; TIM1->PSC = 0; TIM1->ARR = 4250;
  TIM1->CCMR1 = 0x6868; TIM1->CCMR2 = 0x68;
  TIM1->CCER  = 0xFFF; 
  TIM1->BDTR  = 0x80A0; 
  TIM1->CR2   = 0xF3F; TIM1->CR2 &= ~TIM_CR2_MMS_Msk; TIM1->CR2 |= (0x2 << TIM_CR2_MMS_Pos); 
  TIM1->EGR  |= 1; TIM1->CR1 |= 1;
}

// ==========================================
// 4. BARE-METAL ADCs (3-SHUNT INJECTED SCAN)
// ==========================================
void setup_bare_metal_ADCs() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC1EN | RCC_AHB2ENR_ADC12EN;
  GPIOA->MODER |= (3 << 2) | (3 << 8) | (3 << 12) | (3 << 14); GPIOB->MODER |= (3 << 0);
  DAC1->CR |= 1; DAC1->DHR12R1 = 2048;
  ADC12_COMMON->CCR &= ~(3 << 16); ADC12_COMMON->CCR |= (1 << 16);
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL); delay(1);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));
  ADC1->SMPR1 |= (3 << 6); ADC1->SMPR2 |= (3 << 15); ADC2->SMPR1 |= (3 << 12); 
  ADC1->JSQR = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15);
  ADC2->JSQR = (0 << 0) | (1 << 7) | (4 << 9);
  ADC1->CR |= ADC_CR_JADSTART; ADC2->CR |= ADC_CR_JADSTART;
}

// ==========================================
// 5. BLDC AUDIO TONE ENGINE
// ==========================================
void playTone(int freq_hz, int duration_ms) {
  uint32_t period_us = 1000000 / freq_hz;
  uint32_t half_period = period_us / 2;
  uint32_t start_time = millis();
  uint32_t base_ccer = 0xFFF;
  while (millis() - start_time < duration_ms) {
    TIM1->CCR1 = (uint16_t)(0.12f * 4250.0f); TIM1->CCR2 = 0; TIM1->CCR3 = 0;
    TIM1->CCER = base_ccer; delayMicroseconds(half_period);
    TIM1->CCER = 0xAAA; delayMicroseconds(half_period);
  }
  TIM1->CCER = 0xAAA; 
}

// ==========================================
// 6. ELRS CRSF PROTOCOL PARSER
// ==========================================
void readELRS() {
  static uint8_t crsf_buf[64]; static uint8_t crsf_idx = 0; static uint32_t last_packet_ms = 0;
  if (USART1->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
      USART1->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
  }
  while (ELRS_Serial.available()) {
    uint8_t b = ELRS_Serial.read();
    if (crsf_idx == 0 && b != 0xC8) continue; 
    crsf_buf[crsf_idx++] = b;
    if (crsf_idx >= 2 && crsf_idx >= crsf_buf[1] + 2) { 
        if (crsf_buf[2] == 0x16) { 
          last_packet_ms = millis(); 
          uint16_t ch[4];
          ch[0] = (crsf_buf[3] | crsf_buf[4] << 8) & 0x07FF;
          ch[1] = (crsf_buf[4] >> 3 | crsf_buf[5] << 5) & 0x07FF;
          ch[2] = (crsf_buf[5] >> 6 | crsf_buf[6] << 2 | crsf_buf[7] << 10) & 0x07FF;
          float raw_thr = (ch[THROTTLE_CH] - 172) / (1811.0f - 172.0f);
          elrs_throttle = constrain(raw_thr, 0.0f, 1.0f);
          if (elrs_throttle < 0.05f) elrs_throttle = 0.0f;
          
          if (!is_armed && elrs_throttle == 0.0f) {
             is_armed = true;
             Serial.println("ARMING DETECTED! Playing Doo-Wop...");
             playTone(880, 250); delay(50); playTone(1175, 400); 
          }
        }
        crsf_idx = 0; 
    }
    if (crsf_idx >= 64) crsf_idx = 0; 
  }
  if (millis() - last_packet_ms > 200) elrs_throttle = 0.0f; // Failsafe
}

// ==========================================
// 7. SETUP
// ==========================================
void setup() {
  hardware_lockdown(); Serial.begin(115200); delay(2000);
  Serial.println("--- BARE-METAL FOC V2.2 (STATE MACHINE) ---");
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  CORDIC->CSR = (6 << 4) | (1 << 18); // Precision 6, 2 Results
  
  setup_bare_metal_ADCs(); configureTIM1();  
  ELRS_Serial.setRx(PB7); ELRS_Serial.setTx(PB6); ELRS_Serial.begin(420000);

  playTone(1175, 150); playTone(1319, 150); playTone(1397, 150); delay(400);
  int cell_count = (int)(POWER_SUPPLY / 3.7f);
  for (int i = 0; i < cell_count; i++) { playTone(1976, 150); delay(100); }

  // Calibrate
  long sum_U = 0, sum_V = 0, sum_W = 0;
  for (int i = 0; i < 100; i++) { delay(2); sum_U += ADC1->JDR1; sum_V += ADC1->JDR2; sum_W += ADC2->JDR1; }
  offset_U = (int)(sum_U / 100); offset_V = (int)(sum_V / 100); offset_W = (int)(sum_W / 100);
}

// ==========================================
// 8. MAIN LOOP
// ==========================================
void loop() {
  uint32_t now_cycles = DWT->CYCCNT;
  static uint32_t last_cycles = 0;
  float dt = (float)(now_cycles - last_cycles) * (1.0f / 170000000.0f);
  if (dt <= 0.0f || dt > 0.01f) dt = 1e-4f;
  last_cycles = now_cycles;

  readELRS();

  // -------------------------------------------------------
  // STATE 0: IDLE / SAFE COASTING
  // -------------------------------------------------------
  if (!is_armed || elrs_throttle == 0.0f) {
      TIM1->CCER = 0xAAA; // Disconnect FETs
      motor_state = IDLE;
      current_velocity = 0.0f;
      V_alpha_cmd = 0.0f;
      V_beta_cmd = 0.0f;
      return; 
  } else {
      TIM1->CCER = 0xFFF; // Engage FETs
  }

  // -------------------------------------------------------
  // STEP 1: READ CURRENTS
  // -------------------------------------------------------
  float I_U = (float)((int32_t)ADC1->JDR1 - offset_U) * 0.008056f;
  float I_V = (float)((int32_t)ADC1->JDR2 - offset_V) * 0.008056f;
  float I_alpha = I_U;
  float I_beta  = (I_U + 2.0f * I_V) * 0.57735f;  

  // -------------------------------------------------------
  // STEP 2: BEMF OBSERVER (Runs continuously in background)
  // -------------------------------------------------------
  static float I_alpha_prev = 0.0f, I_beta_prev = 0.0f;
  float dIa_dt = (I_alpha - I_alpha_prev) / dt;
  float dIb_dt = (I_beta  - I_beta_prev)  / dt;
  I_alpha_prev = I_alpha; I_beta_prev  = I_beta;

  // BUG FIX: Feed observer the ACTUAL voltage commanded last cycle
  float E_alpha_raw = V_alpha_cmd - (R_phase * I_alpha) - (L_phase * dIa_dt);
  float E_beta_raw  = V_beta_cmd  - (R_phase * I_beta)  - (L_phase * dIb_dt);

  float lpf = dt / (0.002f + dt); // Slightly heavier filter for cleaner signal
  E_alpha = (1.0f - lpf) * E_alpha + lpf * E_alpha_raw;
  E_beta  = (1.0f - lpf) * E_beta  + lpf * E_beta_raw;

  float pure_bemf_angle = atan2f(-E_alpha, E_beta);
  float delta = pure_bemf_angle - last_pure_angle;
  while (delta >  _PI) delta -= _2PI;
  while (delta < -_PI) delta += _2PI;
  last_pure_angle = pure_bemf_angle;
  
  float obs_velocity = delta / dt;
  obs_angle = pure_bemf_angle;

  // -------------------------------------------------------
  // STEP 3: THE STARTUP STATE MACHINE
  // -------------------------------------------------------
  float Vd = 0.0f; 
  float Vq = 0.0f;

  if (motor_state == IDLE) {
      motor_state = ALIGN;
      state_timer = millis();
      electrical_angle = 0.0f;
  } 
  else if (motor_state == ALIGN) {
      // Park the rotor at 0 degrees
      Vd = 1.5f; // Small magnetic flux to hold it
      Vq = 0.0f;
      electrical_angle = 0.0f;
      
      if (millis() - state_timer > 300) { // Hold for 300ms
          motor_state = RAMP;
      }
  }
  else if (motor_state == RAMP) {
      // Blindly spin it up
      Vd = 0.0f;
      Vq = 1.5f; // Fixed, safe torque (prevents 27A spikes)
      current_velocity += 1500.0f * dt; // Accelerate smoothly
      electrical_angle += current_velocity * dt;

      // Handover Condition: Fast enough, and BEMF signal is loud enough
      if (current_velocity > 150.0f && (fabsf(E_alpha) > 0.5f || fabsf(E_beta) > 0.5f)) {
          motor_state = CLOSED_LOOP;
          current_velocity = obs_velocity; // Sync speed
      }
  }
  else if (motor_state == CLOSED_LOOP) {
      // WE ARE FLYING!
      Vq = 1.5f + (elrs_throttle * (POWER_SUPPLY - 1.5f)); 
      
      // Safety clamp: if speed drops to zero under load, trigger restart
      if (fabsf(obs_velocity) < 50.0f) {
          motor_state = ALIGN;
          state_timer = millis();
      }

      current_velocity = (0.95f * current_velocity) + (0.05f * obs_velocity);
      electrical_angle = obs_angle; // Snap to magnetic reality
  }

  // -------------------------------------------------------
  // STEP 4: SPACE VECTOR PWM (SVPWM) MODULATOR
  // -------------------------------------------------------
  float sin_e, cos_e;
  get_Fast_Sin_Cos(electrical_angle, &sin_e, &cos_e);

  V_alpha_cmd = Vd * cos_e - Vq * sin_e; // Save for next loop's observer
  V_beta_cmd  = Vd * sin_e + Vq * cos_e;

  float Va_raw = V_alpha_cmd;
  float Vb_raw = -0.5f * V_alpha_cmd + 0.866025f * V_beta_cmd;
  float Vc_raw = -0.5f * V_alpha_cmd - 0.866025f * V_beta_cmd;

  float v_min = fminf(Va_raw, fminf(Vb_raw, Vc_raw));
  float v_max = fmaxf(Va_raw, fmaxf(Vb_raw, Vc_raw));
  float v_com = -(v_max + v_min) * 0.5f;

  float Va_svpwm = Va_raw + v_com;
  float Vb_svpwm = Vb_raw + v_com;
  float Vc_svpwm = Vc_raw + v_com;

  TIM1->CCR1 = (uint16_t)(constrain((Va_svpwm / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR2 = (uint16_t)(constrain((Vb_svpwm / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR3 = (uint16_t)(constrain((Vc_svpwm / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);

  // -------------------------------------------------------
  // STEP 5: TELEMETRY (10 Hz)
  // -------------------------------------------------------
  static uint32_t last_print_ms = 0;
  if (millis() - last_print_ms > 100) {
    last_print_ms = millis();
    Serial.print("St:");   Serial.print(motor_state);
    Serial.print(",Thr:"); Serial.print(elrs_throttle * 100.0f, 0);
    Serial.print(",Spd:"); Serial.print(current_velocity, 0);
    Serial.print(",I_u:"); Serial.print(I_U, 2);
    Serial.print(",E_a:"); Serial.print(E_alpha, 2);
    Serial.println();
  }
}