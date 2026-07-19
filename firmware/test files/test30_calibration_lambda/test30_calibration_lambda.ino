// =================================================================
// BARE-METAL FLUX LINKAGE (LAMBDA) FINDER - WEACT STM32G431
// Forced Open-Loop Spin | Stator Voltage Decoupling | Lambda Extraction
// =================================================================
#include <Arduino.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

const float POWER_SUPPLY = 12.0f;
const float VOLTS_PER_COUNT = 3.3f / 4095.0f;
const float NOMINAL_AMPS_PER_COUNT = VOLTS_PER_COUNT / (0.002f * 20.0f); // ~0.02014 A/count

// --- KNOWN SYSTEM PARAMETERS ---
const float R_stator = 0.184f;       // Converged from SysID
const float L_stator = 0.0000064f;   // Converged from SysID
const float gain_V   = 0.94f;        // Balanced Scalar

float offset_U = 2079.5f;
float offset_V = 2079.4f;

// --- TUNED OPEN LOOP CONSTANTS TO CRUSH HUNTING RESONANCE ---
const float TARGET_SPEED = 350.0f;   // Bumped up from 150 to let inertia smooth out cogging
float open_loop_angle    = 0.0f;
float open_loop_voltage  = 1.1f;     // Lowered from 1.8V to prevent aggressive current snapping

// ==========================================
// HARDWARE INITIALIZATION
// ==========================================
void hardware_lockdown() {
  pinMode(PA8,  OUTPUT); pinMode(PA9,  OUTPUT); pinMode(PA10, OUTPUT);
  pinMode(PB13, OUTPUT); pinMode(PB14, OUTPUT); pinMode(PB15, OUTPUT);
  digitalWrite(PA8,  DRIVER_OFF); digitalWrite(PB13, DRIVER_OFF);
  digitalWrite(PA9,  DRIVER_OFF); digitalWrite(PB14, DRIVER_OFF);
  digitalWrite(PA10, DRIVER_OFF); digitalWrite(PB15, DRIVER_OFF);
}

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

inline void get_Fast_Sin_Cos(float angle_rad, float* out_sin, float* out_cos) {
  while(angle_rad > _PI) angle_rad -= _2PI;
  while(angle_rad < -_PI) angle_rad += _2PI;
  int32_t angle_q31 = (int32_t)((angle_rad / _PI) * 2147483648.0f);
  CORDIC->WDATA = angle_q31;
  *out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
  *out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
}

// Inverse Park + Inverse Clarke Modulator for direct alpha/beta reference injection
void setInverterVoltages(float v_alpha, float v_beta) {
  float Va_raw = v_alpha;
  float Vb_raw = -0.5f * v_alpha + 0.866025f * v_beta;
  float Vc_raw = -0.5f * v_alpha - 0.866025f * v_beta;

  float v_min = fminf(Va_raw, fminf(Vb_raw, Vc_raw));
  float v_max = fmaxf(Va_raw, fmaxf(Vb_raw, Vc_raw));
  float v_com = -(v_max + v_min) * 0.5f;

  TIM1->CCR1 = (uint16_t)(constrain(((Va_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR2 = (uint16_t)(constrain(((Vb_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR3 = (uint16_t)(constrain(((Vc_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCER = 0xFFF;
}

// ==========================================
// INITIAL SETUP
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(115200);
  while(!Serial);
  delay(1000);

  Serial.println("\n=================================================");
  Serial.println("     FOC KERNEL V2.4 - FLUX LINKAGE FINDER       ");
  Serial.println("=================================================");

  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  CORDIC->CSR = (6 << 4) | (1 << 18); 

  setup_bare_metal_ADCs();
  configureTIM1();
  
  // Set initial stable open-loop alignment state
  setInverterVoltages(0.2f, 0.0f);
  delay(500); 
  
  Serial.println("Spooling up motor in Open-Loop... Please wait.");
}

// ==========================================
// CORE MEASUREMENT LOOP
// ==========================================
void loop() {
  static uint32_t last_cycles = 0;
  uint32_t now_cycles = DWT->CYCCNT;
  float dt = (float)(now_cycles - last_cycles) * (1.0f / 170000000.0f);
  if (dt <= 0.0f || dt > 0.01f) dt = 1e-4f;
  last_cycles = now_cycles;

  // 1. Forced Open-Loop Constant Velocity Spin Architecture
  static float dynamic_speed = 10.0f;
  if (dynamic_speed < TARGET_SPEED) {
    dynamic_speed += 40.0f * dt; // Smoothly ramp velocity up to minimize current rushes
  } else {
    dynamic_speed = TARGET_SPEED;
  }

  open_loop_angle += dynamic_speed * dt;
  while (open_loop_angle >= _2PI) open_loop_angle -= _2PI;

  float sin_ol, cos_ol;
  get_Fast_Sin_Cos(open_loop_angle, &sin_ol, &cos_ol);

  // Inject standard balanced rotating space voltage vector
  float V_alpha = -open_loop_voltage * sin_ol;
  float V_beta  =  open_loop_voltage * cos_ol;
  setInverterVoltages(V_alpha, V_beta);

  // 2. Read Balanced Currents
  float I_U = (float)((int32_t)ADC1->JDR1 - (int)offset_U) * NOMINAL_AMPS_PER_COUNT;
  float I_V = (float)((int32_t)ADC1->JDR2 - (int)offset_V) * NOMINAL_AMPS_PER_COUNT * gain_V;
  
  float I_alpha = I_U;
  float I_beta  = (I_U + 2.0f * I_V) * 0.57735f;

  // 3. High-Bandwidth Stator Voltage Decoupling Math
  static float I_alpha_prev = 0.0f, I_beta_prev = 0.0f;
  float dIa_dt = (I_alpha - I_alpha_prev) / dt;
  float dIb_dt = (I_beta - I_beta_prev) / dt;
  I_alpha_prev = I_alpha;
  I_beta_prev  = I_beta;

  // Compute the unfilterable Back-EMF elements in real-time
  float E_alpha = V_alpha - (R_stator * I_alpha) - (L_stator * dIa_dt);
  float E_beta  = V_beta  - (R_stator * I_beta)  - (L_stator * dIb_dt);

  // Heavily filter the final calculated metrics to isolate purely the steady-state value
  static float E_alpha_filt = 0.0f, E_beta_filt = 0.0f;
  float lpf_g = dt / (0.01f + dt); // 10ms LPF filter window
  E_alpha_filt = (1.0f - lpf_g) * E_alpha_filt + lpf_g * E_alpha;
  E_beta_filt  = (1.0f - lpf_g) * E_beta_filt  + lpf_g * E_beta;

  float E_magnitude = sqrtf(E_alpha_filt * E_alpha_filt + E_beta_filt * E_beta_filt);
  
  // Extract lambda only once the motor speed has fully stabilized on the target platform
  float lambda_sample = 0.0f;
  if (dynamic_speed >= TARGET_SPEED) {
    lambda_sample = E_magnitude / TARGET_SPEED;
  }

  // 4. Print Data out to Negotiate Statistical Consensus
  static uint32_t last_print_ms = 0;
  if (millis() - last_print_ms > 100) {
    last_print_ms = millis();
    if (dynamic_speed < TARGET_SPEED) {
      Serial.print("Spooling... Speed: "); Serial.print(dynamic_speed, 1); Serial.println(" rad/s");
    } else {
      Serial.print("EMF_Mag: ");   Serial.print(E_magnitude, 3);
      Serial.print("V | Speed: "); Serial.print(TARGET_SPEED, 0);
      Serial.print("rad/s | Calculated Lambda: "); Serial.print(lambda_sample, 6);
      Serial.println(" V.s/rad");
    }
  }
}