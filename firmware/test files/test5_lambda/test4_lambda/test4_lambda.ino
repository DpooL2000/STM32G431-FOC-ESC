// =================================================================
// BARE-METAL LAMBDA (FLUX LINKAGE) EXTRACTOR V3.4
// Target: WeAct STM32G431 | Using Confirmed SPWM R & L Values
// =================================================================
#include <Arduino.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

const float POWER_SUPPLY = 12.0f;
const float LOOP_DT      = 50e-6f; // 20kHz loop

// --- HARDWARE CONSTANTS ---
const float SHUNT_RESISTOR  = 0.002f;
const float AMP_GAIN        = 20.0f;
const float VOLTS_PER_COUNT = 3.3f / 4095.0f;
const float NOMINAL_AMPS_PER_COUNT = VOLTS_PER_COUNT / (SHUNT_RESISTOR * AMP_GAIN);

// --- THE "GENIUS" CONSTANTS (From SPWM GUI & SysID) ---
const float R_stator = 0.0930f;      // 93.0 mOhm
const float L_stator = 0.0000468f;   // 46.8 uH

// Boot offsets
float offset_U = 2048.0f; 
float offset_V = 2048.0f;

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
  TIM1->CCMR1 = 0x6868; TIM1->CCMR2 = 0x6868; 
  TIM1->CCR1  = 2125; TIM1->CCR2  = 2125; TIM1->CCR3  = 2125;
  TIM1->CCR4  = 4240; 
  TIM1->CCER  = 0xFFF;
  TIM1->BDTR  = 0x80A0;
  TIM1->CR2   = 0xF3F;
  TIM1->CR2  &= ~TIM_CR2_MMS_Msk;
  TIM1->CR2  |= (0x7 << TIM_CR2_MMS_Pos); 
  TIM1->EGR  |= 1;
  TIM1->CR1  |= 1;
}

void setup_bare_metal_ADCs() {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC3EN | RCC_AHB2ENR_ADC12EN;

  GPIOA->MODER |= (3 << 2) | (3 << 4) | (3 << 8) | (3 << 12) | (3 << 14);
  GPIOB->MODER |= (3 << 0);
  GPIOA->PUPDR &= ~((3 << 4) | (3 << 8) | (3 << 12));

  DAC3->CR &= ~DAC_CR_EN1;
  DAC3->MCR &= ~DAC_MCR_MODE1_Msk;
  DAC3->MCR |= (3 << DAC_MCR_MODE1_Pos); 
  DAC3->CR |= DAC_CR_EN1;
  DAC3->DHR12R1 = 2048; 

  OPAMP1->CSR = (3 << 5) | (3 << 2); 
  __asm__("nop"); __asm__("nop");
  OPAMP1->CSR |= (1 << 0);

  ADC12_COMMON->CCR &= ~(3 << 16); ADC12_COMMON->CCR |= (2 << 16);
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL); delay(1);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));
  
  ADC1->SMPR1 |= (6 << 6); ADC1->SMPR2 |= (6 << 15); ADC2->SMPR1 |= (6 << 12);
  ADC1->JSQR = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15);
  ADC2->JSQR = (0 << 0) | (1 << 7) | (4 << 9);
  
  ADC1->CR |= ADC_CR_JADSTART; ADC2->CR |= ADC_CR_JADSTART;
}

void calibrate_AD8418_offsets() {
  TIM1->CCR1 = 2125; TIM1->CCR2 = 2125; TIM1->CCR3 = 2125;
  delay(200); 

  long sum_U = 0, sum_V = 0;
  for (int i = 0; i < 4000; i++) {
    while (!(TIM1->SR & TIM_SR_UIF)); 
    TIM1->SR &= ~TIM_SR_UIF;           
    sum_U += ADC1->JDR1;  
    sum_V += ADC1->JDR2;  
  }
  offset_U = (float)sum_U / 4000.0f;
  offset_V = (float)sum_V / 4000.0f;
}

inline void get_Fast_Sin_Cos(float angle_rad, float* out_sin, float* out_cos) {
  while(angle_rad > _PI) angle_rad -= _2PI;
  while(angle_rad < -_PI) angle_rad += _2PI;
  int32_t angle_q31 = (int32_t)((angle_rad / _PI) * 2147483648.0f);
  CORDIC->WDATA = angle_q31;
  *out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
  *out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
}

void setInverterVoltages(float v_alpha, float v_beta) {
  float Va_raw = v_alpha;
  float Vb_raw = -0.5f * v_alpha + 0.8660254f * v_beta;
  float Vc_raw = -0.5f * v_alpha - 0.8660254f * v_beta;

  float v_min = fminf(Va_raw, fminf(Vb_raw, Vc_raw));
  float v_max = fmaxf(Va_raw, fmaxf(Vb_raw, Vc_raw));
  float v_com = -(v_max + v_min) * 0.5f; 

  TIM1->CCR1 = (uint16_t)(constrain(((Va_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR2 = (uint16_t)(constrain(((Vb_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR3 = (uint16_t)(constrain(((Vc_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCER = 0xFFF; 
}

// ==========================================
// MAIN
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(115200); 
  delay(1000);

  Serial.println("\n=================================================");
  Serial.println("     LAMBDA (FLUX LINKAGE) EXTRACTOR V3.4        ");
  Serial.println("=================================================");

  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  CORDIC->CSR = (6 << 4) | (1 << 18); 

  setup_bare_metal_ADCs();
  configureTIM1();
  calibrate_AD8418_offsets();
  
  Serial.println("Offsets locked. Ramping up Open-Loop Spin...");

  GPIOA->MODER  = (GPIOA->MODER  & ~0x3F0000)   | 0x2A0000;
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF)      | 0x666;
  GPIOB->MODER  = (GPIOB->MODER  & ~0xFC000000) | 0xA8000000;
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000;
}

void loop() {
  static uint32_t tick_counter = 0;
  static float speed_ramp = 0.0f;
  static float angle = 0.0f;
  const float target_speed = 350.0f;

  while (!(TIM1->SR & TIM_SR_UIF));
  TIM1->SR &= ~TIM_SR_UIF; 

  float I_U = (float)((int32_t)ADC1->JDR1 - (int)offset_U) * NOMINAL_AMPS_PER_COUNT;
  float I_V = (float)((int32_t)ADC1->JDR2 - (int)offset_V) * NOMINAL_AMPS_PER_COUNT;
  float I_alpha = I_U;
  float I_beta  = (I_U + 2.0f * I_V) * 0.57735f;

  if (speed_ramp < target_speed) speed_ramp += 40.0f * LOOP_DT;
  else speed_ramp = target_speed;

  angle += speed_ramp * LOOP_DT;
  if (angle > _2PI) angle -= _2PI;

  float sin_ol, cos_ol;
  get_Fast_Sin_Cos(angle, &sin_ol, &cos_ol);

  // V/f ramp voltage
  float v_mag = 0.8f + (speed_ramp * 0.006f); 
  float v_alpha = -v_mag * sin_ol;
  float v_beta  =  v_mag * cos_ol;
  setInverterVoltages(v_alpha, v_beta);

  // Extract Lambda when speed is stable
  if (speed_ramp >= target_speed) {
    static float prev_Ia = 0, prev_Ib = 0;
    float dIa_dt = (I_alpha - prev_Ia) / LOOP_DT;
    float dIb_dt = (I_beta - prev_Ib) / LOOP_DT;
    prev_Ia = I_alpha; prev_Ib = I_beta;

    // Decouple the stator impedance using your exact R and L
    float E_alpha = v_alpha - (R_stator * I_alpha) - (L_stator * dIa_dt);
    float E_beta  = v_beta  - (R_stator * I_beta)  - (L_stator * dIb_dt);

    static float E_a_filt = 0, E_b_filt = 0;
    float filter_g = LOOP_DT / (0.01f + LOOP_DT); // 10ms LPF
    E_a_filt = (1.0f - filter_g) * E_a_filt + filter_g * E_alpha;
    E_b_filt = (1.0f - filter_g) * E_b_filt + filter_g * E_beta;

    float E_mag = sqrtf(E_a_filt*E_a_filt + E_b_filt*E_b_filt);
    float current_lambda = E_mag / target_speed;

    if (tick_counter % 2000 == 0) { // Print every 100ms
       Serial.print("Target Speed Reached. Lambda Stabilizing -> "); 
       Serial.println(current_lambda, 6);
    }
    
    // Stop after 3 seconds of solid measurement
    if (tick_counter > 80000) { 
      setInverterVoltages(0.0f, 0.0f);
      TIM1->CCER = 0xAAA; // Disable outputs
      Serial.println("\n--- FINAL PARAMETERS ACQUIRED ---");
      Serial.print("R = "); Serial.println(R_stator, 4);
      Serial.print("L = "); Serial.println(L_stator, 7);
      Serial.print("LAMBDA = "); Serial.println(current_lambda, 6);
      Serial.println("\nReady to drop into V3.2 Flux Observer!");
      while(1); // Halt
    }
  }
  tick_counter++;
}