// =================================================================
// BARE-METAL di/dt INDUCTANCE MEASUREMENT (DECAY SLOPE METHOD)
// Target: WeAct STM32G431
// Captures exponential turn-off decay across Phase U and Phase W
// =================================================================
#include <Arduino.h>

#define DRIVER_OFF HIGH
const float POWER_SUPPLY = 12.0f;

// --- IDENTIFIED HARDWARE CONSTANTS ---
const float RS                      = 0.088560f; // Phase Resistance
const float SHUNT_RESISTOR          = 0.002f;
const float AMP_GAIN                = 20.0f;
const float VOLTS_PER_COUNT         = 3.3f / 4095.0f;
const float NOMINAL_AMPS_PER_COUNT  = VOLTS_PER_COUNT / (SHUNT_RESISTOR * AMP_GAIN);

float offset_U = 2048.0f;
float offset_W = 2048.0f;

// Telemetry structure
struct Telemetry {
  uint16_t sync = 0xABCD;
  float iu;
  float iv;
  float iw;
  float i_alpha;
  float i_beta;
  float l_live;
  float l_min;
} __attribute__((packed));

Telemetry t_data;

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
  TIM1->CCR1  = 2125; TIM1->CCR2 = 2125; TIM1->CCR3 = 2125; TIM1->CCR4 = 4240;
  TIM1->CCER  = 0xFFF; TIM1->BDTR = 0x80A0;
  TIM1->CR2   = 0xF3F; TIM1->CR2 &= ~TIM_CR2_MMS_Msk;
  TIM1->CR2  |= (0x7 << TIM_CR2_MMS_Pos);
  TIM1->EGR  |= 1; TIM1->CR1 |= 1;
}

void setup_bare_metal_ADCs() {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC3EN | RCC_AHB2ENR_ADC12EN;

  // PA1=analog, PA2=Analog (for OPAMP1), PA4=analog, PA6=analog, PA7=analog
  GPIOA->MODER |= (3 << 2) | (3 << 4) | (3 << 8) | (3 << 12) | (3 << 14);
  GPIOB->MODER |= (3 << 0);
  GPIOA->PUPDR &= ~((3 << 4) | (3 << 8) | (3 << 12));

  // OPAMP1 / DAC3 setup for 1.65V Reference
  DAC3->CR &= ~DAC_CR_EN1; DAC3->MCR &= ~DAC_MCR_MODE1_Msk; DAC3->MCR |= (3 << DAC_MCR_MODE1_Pos);
  DAC3->CR |= DAC_CR_EN1; DAC3->DHR12R1 = 2048;
  OPAMP1->CSR = (3 << 5) | (3 << 2); __asm__("nop"); __asm__("nop"); OPAMP1->CSR |= (1 << 0);

  ADC12_COMMON->CCR &= ~(3 << 16);
  ADC12_COMMON->CCR |= (3 << 16); // AHB/4 clock fix
  
  // WAKE UP ADC
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);

  // WIPE HARDWARE OFFSETS TO PREVENT GHOST DRIFT
  ADC1->OFR1 = 0; ADC1->OFR2 = 0; ADC2->OFR1 = 0;

  // CALIBRATE
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL); delay(1);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));

  ADC1->SMPR1 |= (6 << 6); ADC1->SMPR2 |= (6 << 15); ADC2->SMPR1 |= (6 << 12);
  
  // STRICT MAPPING: ADC1 reads U (PB0/CH15). ADC2 reads W (PA7/CH4).
  ADC1->JSQR = (0 << 0) | (1 << 7) | (15 << 9); 
  ADC2->JSQR = (0 << 0) | (1 << 7) | (4 << 9);
  
  ADC1->CR |= ADC_CR_JADSTART; ADC2->CR |= ADC_CR_JADSTART;
}

// ==========================================
// ZERO-CURRENT OFFSET CALIBRATION
// ==========================================
void calibrate_AD8418_offsets() {
  uint32_t bdtr_backup = TIM1->BDTR;
  TIM1->BDTR &= ~TIM_BDTR_MOE; // Disable outputs completely
  delay(200); 

  long sum_U = 0, sum_W = 0;
  const int N = 2000;
  for (int i = 0; i < N; i++) {
    while (!(TIM1->SR & TIM_SR_UIF)); TIM1->SR &= ~TIM_SR_UIF;
    sum_U += ADC1->JDR1;
    sum_W += ADC2->JDR1;
  }
  offset_U = (float)sum_U / (float)N;
  offset_W = (float)sum_W / (float)N;
  TIM1->BDTR = bdtr_backup; // Re-enable outputs
}

// ==========================================
// VOLTAGE OUTPUT (U AND W INJECTION)
// ==========================================
void setTestVoltageUW(float v_applied) {
  float v_delta = v_applied / POWER_SUPPLY;
  float duty_U  = 0.5f + (v_delta * 0.5f);
  float duty_W  = 0.5f - (v_delta * 0.5f);
  
  TIM1->CCR1 = (uint16_t)(constrain(duty_U, 0.05f, 0.95f) * 4250.0f); // Phase U
  TIM1->CCR3 = (uint16_t)(constrain(duty_W, 0.05f, 0.95f) * 4250.0f); // Phase W
  TIM1->CCR2 = 2125;                                                  // Phase V Neutral
  TIM1->CCER = 0xFFF;
}

// ==========================================
// SIMULTANEOUS CURRENT READBACK
// ==========================================
inline void getCalibratedCurrents(float* i_u_out, float* i_v_out, float* i_w_out) {
  float raw_u = (float)((int32_t)ADC1->JDR1 - (int)offset_U);
  float raw_w = (float)((int32_t)ADC2->JDR1 - (int)offset_W);
  
  *i_u_out = raw_u * NOMINAL_AMPS_PER_COUNT;
  *i_w_out = raw_w * NOMINAL_AMPS_PER_COUNT;
  *i_v_out = -(*i_u_out + *i_w_out); // Reconstruct V perfectly
}

// ==========================================
// MAIN SETUP
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(2000000);
  delay(1000);

  setup_bare_metal_ADCs();
  configureTIM1();
  calibrate_AD8418_offsets();
  
  // Re-apply alternate functions
  GPIOA->MODER  = (GPIOA->MODER  & ~0x3F0000)   | 0x2A0000;
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF)      | 0x666;
  GPIOB->MODER  = (GPIOB->MODER  & ~0xFC000000) | 0xA8000000;
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000;
}

// ==========================================
// MAIN LOOP: 20kHz STATE MACHINE
// ==========================================
void loop() {
  static uint32_t loop_tick = 0;
  static float i_start = 0.0f;
  static float i_end = 0.0f;
  static float current_L = 0.0f;
  
  // 1. HARDWARE SYNC (20kHz = 50us per tick)
  while (!(TIM1->SR & TIM_SR_UIF));
  TIM1->SR &= ~TIM_SR_UIF; 

  float iu, iv, iw;
  getCalibratedCurrents(&iu, &iv, &iw);

  // 2. THE 400ms DECAY STATE MACHINE 
  // Each state is exactly 100ms (2000 ticks)
  
  if (loop_tick == 0) {
      setTestVoltageUW(1.0f); // Apply +1.0V (Current rises to ~5A)
  } 
  else if (loop_tick == 1999) {
      i_start = iu;           // Capture peak current RIGHT BEFORE turn-off
  }
  else if (loop_tick == 2000) {
      setTestVoltageUW(0.0f); // Turn off voltage -> DECAY BEGINS!
  }
  else if (loop_tick == 2002) {
      i_end = iu;             // Capture current 2 ticks (100us) into the decay
      
      float abs_start = fabs(i_start);
      float abs_end = fabs(i_end);
      
      // Calculate L using the exponential decay logarithm
      if (abs_start > 0.5f && abs_end < abs_start) {
          float delta_t = 100e-6f; // 100 microseconds
          float ln_ratio = logf(abs_end / abs_start); 
          current_L = (-RS * delta_t) / ln_ratio; // Exact physical Inductance
      }
  }
  else if (loop_tick == 4000) {
      setTestVoltageUW(-1.0f); // Apply -1.0V (Current falls to ~ -5A)
  }
  else if (loop_tick == 5999) {
      i_start = iu;            // Capture peak negative current RIGHT BEFORE turn-off
  }
  else if (loop_tick == 6000) {
      setTestVoltageUW(0.0f);  // Turn off voltage -> DECAY BEGINS!
  }
  else if (loop_tick == 6002) {
      i_end = iu;              // Capture current 2 ticks (100us) into the decay
      
      float abs_start = fabs(i_start);
      float abs_end = fabs(i_end);
      
      if (abs_start > 0.5f && abs_end < abs_start) {
          float delta_t = 100e-6f; 
          float ln_ratio = logf(abs_end / abs_start);
          current_L = (-RS * delta_t) / ln_ratio;
      }
  }

  // 3. TELEMETRY DUMP (Decimated to 1kHz)
  if (loop_tick % 20 == 0) {
      t_data.iu = iu;
      t_data.iv = iv;
      t_data.iw = iw;
      t_data.i_alpha = iu;
      t_data.i_beta  = -(iu + 2.0f * iw) * 0.577350269f;
      t_data.l_live  = current_L * 1000000.0f; // Send to GUI in microHenries
      t_data.l_min   = 0.0f;
      
      Serial.write((uint8_t*)&t_data, sizeof(Telemetry));
  }

  // 4. ADVANCE STATE
  loop_tick++;
  if (loop_tick >= 8000) loop_tick = 0; // Total cycle = 400ms
}