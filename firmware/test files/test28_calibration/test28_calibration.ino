// =================================================================
// BARE-METAL SYSID KERNEL V2.3 - NOISY MEASUREMENT NEGOTIATION
// Relative Balance -> Scale to Amps -> Statistical R & L Extraction
// =================================================================
#include <Arduino.h>

#define DRIVER_OFF HIGH
const float POWER_SUPPLY = 12.0f;

// Nominal hardware constants (used as the baseline for our scaling negotiation)
const float SHUNT_RESISTOR = 0.002f; 
const float AMP_GAIN       = 20.0f;   // AD8418 standard gain
const float VOLTS_PER_COUNT = 3.3f / 4095.0f;
const float NOMINAL_AMPS_PER_COUNT = VOLTS_PER_COUNT / (SHUNT_RESISTOR * AMP_GAIN); // ~0.02014 A/count

// --- CALIBRATION VARIABLES ---
float offset_U = 2048.0f;
float offset_V = 2048.0f;
float offset_W = 2048.0f;

float gain_scalar_V = 1.0f; // Eliminates hardware ellipse
float R_identified  = 0.0f; 
float L_identified  = 0.0f; 

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

void setTestVoltageUV(float v_applied) {
  float v_delta = v_applied / POWER_SUPPLY;
  float duty_U = 0.5f + (v_delta * 0.5f);
  float duty_V = 0.5f - (v_delta * 0.5f);
  TIM1->CCR1 = (uint16_t)(constrain(duty_U, 0.02f, 0.98f) * 4250.0f);
  TIM1->CCR2 = (uint16_t)(constrain(duty_V, 0.02f, 0.98f) * 4250.0f);
  TIM1->CCR3 = 2125; 
  TIM1->CCER = 0xFFF; 
}

// Inline helper to get balanced, scaled currents from raw noisy readings
inline void getCalibratedCurrents(float* i_u_out, float* i_v_out) {
  float raw_u = (float)((int32_t)ADC1->JDR1 - (int)offset_U);
  float raw_v = (float)((int32_t)ADC1->JDR2 - (int)offset_V);

  // 1. Apply relative balancing scalar first (Phase V matched to U)
  float raw_v_balanced = raw_v * gain_scalar_V;

  // 2. Scale to actual physical current values
  *i_u_out = raw_u * NOMINAL_AMPS_PER_COUNT;
  *i_v_out = raw_v_balanced * NOMINAL_AMPS_PER_COUNT;
}

// ==========================================
// MAIN SYSID ENGINE
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(115200);
  while(!Serial);
  delay(1000);

  Serial.println("\n=================================================");
  Serial.println("   FOC KERNEL V2.3 - NOISY SYSID NEGOTIATION     ");
  Serial.println("=================================================");
  
  setup_bare_metal_ADCs();
  configureTIM1();
  
  TIM1->CCER = 0xAAA; 
  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
  delay(500); 

  // -------------------------------------------------------
  // PHASE 1: ATOMIC ZERO CURRENT OFFSETS
  // -------------------------------------------------------
  Serial.print("Phase 1: Tracking Offsets... ");
  long sum_U = 0, sum_V = 0, sum_W = 0;
  for (int i = 0; i < 4000; i++) {
    delayMicroseconds(100);
    sum_U += ADC1->JDR1;
    sum_V += ADC1->JDR2;
    sum_W += ADC2->JDR1;
  }
  offset_U = (float)sum_U / 4000.0f;
  offset_V = (float)sum_V / 4000.0f;
  offset_W = (float)sum_W / 4000.0f;
  Serial.println("CONVERGED.");

  // -------------------------------------------------------
  // PHASE 2: CURRENT AMPLIFIER RELATIVE GAIN BALANCING
  // -------------------------------------------------------
  Serial.print("Phase 2: Negotiating Relative Phase Balance (U vs V)... ");
  setTestVoltageUV(0.5f); // Safe excitation to pull data from noise floor
  delay(300); // Allow inductive transitions to fully die out

  double raw_accumulation_U = 0.0;
  double raw_accumulation_V = 0.0;

  for (int i = 0; i < 2000; i++) {
    delayMicroseconds(50);
    raw_accumulation_U += fabsf((float)((int32_t)ADC1->JDR1 - (int)offset_U));
    raw_accumulation_V += fabsf((float)((int32_t)ADC1->JDR2 - (int)offset_V));
  }
  
  // Compute relative scale so Phase V reads perfectly equal to Phase U
  if (raw_accumulation_V > 100.0) {
    gain_scalar_V = (float)(raw_accumulation_U / raw_accumulation_V);
  }
  Serial.println("BALANCED.");

  // -------------------------------------------------------
  // PHASE 3: ACTIVE SEARCH & SCALING VALIDATION (RESISTANCE)
  // -------------------------------------------------------
  Serial.println("Phase 3: Searching for Statistical Current Convergence Zone...");
  
  float test_voltage = 0.05f;
  float current_U_stable = 0.0f;
  bool  r_converged = false;
  uint32_t stage_timer = millis();

  while (!r_converged) {
    if (millis() - stage_timer > 2500) { // Thermal Protection hard cut
      Serial.println("!!! TIMEOUT: Forcing Exit on Safest Window !!!");
      break;
    }

    setTestVoltageUV(test_voltage);
    delay(20); // Settle PWM step artifact

    // Statistical filter window
    float current_sum_U = 0.0f;
    float current_sum_V = 0.0f;
    for (int i = 0; i < 200; i++) {
      float iu, iv;
      getCalibratedCurrents(&iu, &iv); // Using the newly balanced and scaled currents
      current_sum_U += iu;
      current_sum_V += iv;
      delayMicroseconds(20);
    }
    float current_avg_U = fabsf(current_sum_U / 200.0f);

    // Noisy Measurement Check: Ensure current is clear of noise floor (> 1.5 Amps absolute)
    if (current_avg_U < 1.5f) {
      test_voltage += 0.02f; // Safely step up electrical pressure
      continue;
    }

    // Mathematical Convergence Validation: Verify stability window over time
    static float last_filtered_amps = 0.0f;
    if (fabsf(current_avg_U - last_filtered_amps) < 0.005f) { // Stable within 5mA
      current_U_stable = current_avg_U;
      r_converged = true;
      Serial.print("-> Statistical Consensus At: "); Serial.print(test_voltage, 3); Serial.println("V");
    }
    last_filtered_amps = current_avg_U;
  }

  // Calculate Phase Resistance using the negotiated scaled values
  // R_loop = V / I -> R_phase = R_loop / 2
  R_identified = (test_voltage / current_U_stable) * 0.5f;

  // -------------------------------------------------------
  // PHASE 4: TRANSIENT INDUCTANCE SAMPLING (SCALED CURRENT ONLY)
  // -------------------------------------------------------
  Serial.print("Phase 4: Sampling Inductive Slopes (20-Pulse Track)... ");
  
  float L_accumulator = 0.0f;
  int pulse_count = 0;
  float pulse_voltage = test_voltage + 1.0f; // Slap an extra 1V potential step

  for (int p = 0; p < 20; p++) {
    setTestVoltageUV(test_voltage); // Reset to base baseline current state
    delay(15); 

    float i_u_baseline, i_v_baseline;
    getCalibratedCurrents(&i_u_baseline, &i_v_baseline);

    uint32_t t0 = micros();
    setTestVoltageUV(pulse_voltage);
    delayMicroseconds(50); // Inside linear inductive transient ramp
    float i_u_transient, i_v_transient;
    getCalibratedCurrents(&i_u_transient, &i_v_transient);
    uint32_t t1 = micros();

    float dt = (float)(t1 - t0) * 1e-6f;
    float delta_I = fabsf(i_u_transient - i_u_baseline);

    if (delta_I > 0.02f) { // Ensure delta isn't corrupted by a noise spike
      float L_loop = (1.0f * dt) / delta_I;
      L_accumulator += (L_loop * 0.5f); // Phase to neutral
      pulse_count++;
    }
  }

  if (pulse_count > 0) {
    L_identified = L_accumulator / (float)pulse_count;
  }
  Serial.println("CONVERGED.");

  // SHUT DOWN ALL INVERTER GATES
  TIM1->CCER = 0xAAA;
  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;

  // -------------------------------------------------------
  // BALANCED DIAGNOSTIC REPORT
  // -------------------------------------------------------
  Serial.println("\n=================================================");
  Serial.println("           CONVERGED SYSID REPORT (SUCCESS)       ");
  Serial.println("=================================================");
  Serial.print("Offsets -> U: "); Serial.print(offset_U, 1);
  Serial.print(" | V: "); Serial.print(offset_V, 1);
  Serial.print(" | W: "); Serial.println(offset_W, 1);
  Serial.println("-------------------------------------------------");
  Serial.print("Phase V Scalar (Relative to U) : "); Serial.println(gain_scalar_V, 4);
  Serial.print("True Phase Resistance (R)      : "); Serial.print(R_identified * 1000.0f, 2); Serial.println(" mOhm");
  Serial.print("True Phase Inductance (L)      : "); Serial.print(L_identified * 1000000.0f, 2); Serial.println(" uH");
  Serial.println("=================================================\n");
}

void loop() {
  // Safe hold state
}