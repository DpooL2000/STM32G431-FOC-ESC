/**
 * 6-Step Commutation with FOC Fwd Math Diagnostic
 * Pins: U = PB0, V = PA1, W = PA7
 * Features: Fixed CORDIC Math, Dynamic Calibration, Non-Blocking Loop
 */

#include <Arduino.h>

// --- GLOBALS ---
int step = 1;
int pwmValue = 850; 
int stepDelay = 4;  
unsigned long lastStepTime = 0;
unsigned long lastPrintTime = 0;

// Hardcoded physical offsets (as baselines before dynamic scaling)
const float offset_V = 0.0696; 
const float offset_W = 0.0686; 
const float offset_U = 0.0686; 

// --- FAST CORDIC TRIG (FIXED) ---
inline void get_Fast_Sin_Cos(float angle_rad, float* out_sin, float* out_cos) {
  while(angle_rad > PI) angle_rad -= 2.0f * PI;
  while(angle_rad < -PI) angle_rad += 2.0f * PI;
  
  int32_t angle_q31 = (int32_t)((angle_rad / PI) * 2147483648.0f);
  CORDIC->WDATA = angle_q31;
  
  *out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
  *out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // WAKE UP CORDIC WITH CORRECT REGISTERS
  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  CORDIC->CSR = (6 << CORDIC_CSR_PRECISION_Pos) | CORDIC_CSR_NRES; 

  // BARE METAL OPAMP & GPIO SETUP
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  
  // Set PA1, PA7, PB0 to Analog Mode
  GPIOA->MODER |= (3 << (1 * 2)) | (3 << (7 * 2));  
  GPIOB->MODER |= (3 << (0 * 2));  

  // OPAMP Setup (16x Gain)
  OPAMP1->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);
  OPAMP2->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);
  OPAMP3->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);

  // TIMER 1 SETUP (For Motor PWM)
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
  GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
  GPIOA->AFR[1] &= ~((0xF << 0) | (0xF << 4) | (0xF << 8));
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 

  TIM1->PSC = 0;           
  TIM1->ARR = 8500 - 1;    
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
  TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M);
  TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
  TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos);
  
  TIM1->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  TIM1->BDTR |= TIM_BDTR_MOE;

  TIM1->CR1 |= TIM_CR1_CEN;
}

void loop() {
  // 1. READ RAW PINS (Directly from PB0, PA1, PA7 as requested)
  int rawU = analogRead(PB0); 
  int rawV = analogRead(PA1); 
  int rawW = analogRead(PA7); 

  float voltsU = (rawU / 4095.0) * 3.3;
  float voltsV = (rawV / 4095.0) * 3.3;
  float voltsW = (rawW / 4095.0) * 3.3;

  float ampsU = (voltsU - offset_U) / 0.16;
  float ampsV = (voltsV - offset_V) / 0.16;
  float ampsW = (voltsW - offset_W) / 0.16;

  // 2. DYNAMIC CALIBRATION & NORMALIZATION
  static float maxU = -10.0, minU = 10.0;
  static float maxV = -10.0, minV = 10.0;
  static float maxW = -10.0, minW = 10.0;
  float decay = 0.002; 

  if (ampsU > maxU) maxU = ampsU; else maxU -= decay;
  if (ampsU < minU) minU = ampsU; else minU += decay;
  if (ampsV > maxV) maxV = ampsV; else maxV -= decay;
  if (ampsV < minV) minV = ampsV; else minV += decay;
  if (ampsW > maxW) maxW = ampsW; else maxW -= decay;
  if (ampsW < minW) minW = ampsW; else minW += decay;

  float centerU = (maxU + minU) / 2.0;
  float centerV = (maxV + minV) / 2.0;
  float centerW = (maxW + minW) / 2.0;

  float amplitudeU = maxU - minU;
  float amplitudeV = maxV - minV;
  float amplitudeW = maxW - minW;

  if(amplitudeU < 0.01) amplitudeU = 0.01;
  if(amplitudeV < 0.01) amplitudeV = 0.01;
  if(amplitudeW < 0.01) amplitudeW = 0.01;

  float scaleV = amplitudeU / amplitudeV;
  float scaleW = amplitudeU / amplitudeW;

  float finalU = ampsU - centerU;
  float finalV = (ampsV - centerV) * scaleV;
  float finalW = (ampsW - centerW) * scaleW;

  // 3. FOC DIAGNOSTIC MATH (Clarke & Park)
  // Synthesize a smooth Electrical Angle based on time and step
  float angle_per_step = PI / 3.0f; // 60 degrees
  float fraction_of_step = (millis() - lastStepTime) / (float)stepDelay;
  if (fraction_of_step > 1.0f) fraction_of_step = 1.0f;
  float electrical_angle = ((step - 1) * angle_per_step) + (fraction_of_step * angle_per_step);

  // Execute Fixed Hardware Trig
  float sin_theta, cos_theta;
  get_Fast_Sin_Cos(electrical_angle, &sin_theta, &cos_theta);

  // True 3-Phase Clarke Transform
  float I_alpha = 0.666666f * finalU - 0.333333f * (finalV + finalW);
  float I_beta  = 0.577350f * (finalV - finalW);

  // Park Transform
  float I_d = (I_alpha * cos_theta) + (I_beta * sin_theta);
  float I_q = (I_beta * cos_theta)  - (I_alpha * sin_theta);

  // 4. PRINTING ALL 5 WAVES
  if (millis() - lastPrintTime >= 5) {
    lastPrintTime = millis();
    Serial.print("U:"); Serial.print(finalU, 3); Serial.print(",");
    Serial.print("V:"); Serial.print(finalV, 3); Serial.print(",");
    Serial.print("W:"); Serial.print(finalW, 3); Serial.print(",");
    Serial.print("I_d:"); Serial.print(I_d, 3); Serial.print(",");
    Serial.print("I_q:"); Serial.println(I_q, 3);
  }

  // 5. 6-STEP COMMUTATION TIMING
  if (millis() - lastStepTime >= stepDelay) {
    lastStepTime = millis();

    switch(step) {
      case 1: TIM1->CCR1=pwmValue; TIM1->CCR2=0;        TIM1->CCR3=0;        break;
      case 2: TIM1->CCR1=pwmValue; TIM1->CCR2=pwmValue; TIM1->CCR3=0;        break;
      case 3: TIM1->CCR1=0;        TIM1->CCR2=pwmValue; TIM1->CCR3=0;        break;
      case 4: TIM1->CCR1=0;        TIM1->CCR2=pwmValue; TIM1->CCR3=pwmValue; break;
      case 5: TIM1->CCR1=0;        TIM1->CCR2=0;        TIM1->CCR3=pwmValue; break;
      case 6: TIM1->CCR1=pwmValue; TIM1->CCR2=0;        TIM1->CCR3=pwmValue; break;
    }

    step++;
    if(step > 6) step = 1; 
  }
}