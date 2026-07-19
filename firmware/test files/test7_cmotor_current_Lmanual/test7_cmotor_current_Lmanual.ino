// =================================================================
// BARE-METAL CURRENT PULSE GENERATOR (LOW-NOISE EDITION)
// Modified to output HFI telemetry struct for GUI
// =================================================================
#include <Arduino.h>

#define DRIVER_OFF HIGH
const float POWER_SUPPLY = 12.0f;

// --- IDENTIFIED HARDWARE CONSTANTS ---
const float SHUNT_RESISTOR          = 0.002f;
const float AMP_GAIN                = 20.0f;
const float VOLTS_PER_COUNT         = 3.3f / 4095.0f;
const float NOMINAL_AMPS_PER_COUNT  = VOLTS_PER_COUNT / (SHUNT_RESISTOR * AMP_GAIN);

// Non-const so boot calibrator can overwrite them
float offset_U = 2103.6f;
float offset_V = 2103.6f;
float offset_W = 2103.6f;

// Telemetry structure – identical to HFI version
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
// HARDWARE INITIALIZATION (unchanged)
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

  TIM1->CR1   = 0x20;
  TIM1->PSC   = 0;
  TIM1->ARR   = 4250;
  TIM1->CCMR1 = 0x6868;
  TIM1->CCMR2 = 0x6868;
  TIM1->CCR1  = 2125;
  TIM1->CCR2  = 2125;
  TIM1->CCR3  = 2125;
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

  ADC12_COMMON->CCR &= ~(3 << 16);
  ADC12_COMMON->CCR |= (2 << 16);
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL);
  delay(1);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));

  ADC1->SMPR1 |= (6 << 6); ADC1->SMPR2 |= (6 << 15); ADC2->SMPR1 |= (6 << 12);
  ADC1->JSQR = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15);
  ADC2->JSQR = (0 << 0) | (1 << 7) | (4 << 9);
  ADC1->CR |= ADC_CR_JADSTART; ADC2->CR |= ADC_CR_JADSTART;
}

// ==========================================
// BOOT CALIBRATION (unchanged)
// ==========================================
void calibrate_AD8418_offsets() {
  Serial.println("Calibrating offsets... Keep motor still, no current flowing.");
  TIM1->CCR1 = 2125; TIM1->CCR2 = 2125; TIM1->CCR3 = 2125;
  delay(200);
  long sum_U = 0, sum_V = 0, sum_W = 0;
  const int N = 2000;
  for (int i = 0; i < N; i++) {
    while (!(TIM1->SR & TIM_SR_UIF)); TIM1->SR &= ~TIM_SR_UIF;
    sum_U += ADC1->JDR1;
    sum_V += ADC1->JDR2;
    sum_W += ADC2->JDR1;
  }
  offset_U = (float)sum_U / (float)N;
  offset_V = (float)sum_V / (float)N;
  offset_W = (float)sum_W / (float)N;
  Serial.print("Offset U: "); Serial.println(offset_U, 2);
  Serial.print("Offset V: "); Serial.println(offset_V, 2);
  Serial.print("Offset W: "); Serial.println(offset_W, 2);
  Serial.println("Calibration done.");
}

// ==========================================
// VOLTAGE OUTPUT (unchanged)
// ==========================================
void setTestVoltageUV(float v_applied) {
  float v_delta = v_applied / POWER_SUPPLY;
  float duty_U  = 0.5f + (v_delta * 0.5f);
  float duty_V  = 0.5f - (v_delta * 0.5f);
  TIM1->CCR3 = (uint16_t)(constrain(duty_U, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR2 = (uint16_t)(constrain(duty_V, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR1 = 2125;
  TIM1->CCER = 0xFFF;
}

// ==========================================
// CURRENT READBACK
// ==========================================
inline void getCalibratedCurrents(float* i_u_out, float* i_v_out, float* i_w_out) {
  float raw_u = (float)((int32_t)ADC1->JDR1 - (int)offset_U);
  float raw_v = (float)((int32_t)ADC1->JDR2 - (int)offset_V);
  float raw_w = (float)((int32_t)ADC2->JDR1 - (int)offset_W);
  *i_u_out = raw_u * NOMINAL_AMPS_PER_COUNT;
  *i_v_out = raw_v * NOMINAL_AMPS_PER_COUNT;
  *i_w_out = raw_w * NOMINAL_AMPS_PER_COUNT;
}

// ==========================================
// MAIN – now outputs telemetry at 1kHz
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(2000000);      // Match GUI baud rate
  delay(1000);

  setup_bare_metal_ADCs();
  configureTIM1();

  // Calibrate after timer is running
  calibrate_AD8418_offsets();

  // Re‑apply timer pin modes (calibration didn't touch them)
  GPIOA->MODER  = (GPIOA->MODER  & ~0x3F0000)   | 0x2A0000;
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF)      | 0x666;
  GPIOB->MODER  = (GPIOB->MODER  & ~0xFC000000) | 0xA8000000;
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000;

  Serial.println("PULSE GENERATOR ACTIVE – sending telemetry.");
}

void loop() {
  static uint32_t last_telemetry = 0;
  static int pulse_state = 0;
  static uint32_t last_pulse_change = 0;
  uint32_t now = millis();

  // Change pulse state every 100 ms (same as before)
  if (now - last_pulse_change >= 100) {
    last_pulse_change = now;
    pulse_state = (pulse_state + 1) % 4;
    if      (pulse_state == 0) setTestVoltageUV( 1.0f);
    else if (pulse_state == 1) setTestVoltageUV( 0.0f);
    else if (pulse_state == 2) setTestVoltageUV(-1.0f);
    else if (pulse_state == 3) setTestVoltageUV( 0.0f);
  }

  // Send telemetry every 1 ms (1 kHz) – matches HFI code rate
  if (now - last_telemetry >= 1) {
    last_telemetry = now;

    float iu, iv, iw;
    getCalibratedCurrents(&iu, &iv, &iw);

    // Compute Clarke transform
    float i_alpha = iu;
    float i_beta  = -(iu + 2.0f * iw) * 0.577350269f;

    // Fill telemetry – l_live and l_min set to 0 (not used in this test)
    t_data.iu = iu;
    t_data.iv = iv;
    t_data.iw = iw;
    t_data.i_alpha = i_alpha;
    t_data.i_beta  = i_beta;
    t_data.l_live  = 0.0f;
    t_data.l_min   = 0.0f;

    Serial.write((uint8_t*)&t_data, sizeof(Telemetry));
  }
}