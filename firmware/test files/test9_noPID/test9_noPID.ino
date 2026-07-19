#include <SimpleFOC.h>

const int BEMF_A = PA0, BEMF_B = PA3, BEMF_C = PB1;
const float POWER_SUPPLY = 12.0f;
const float SQRT3_2 = 0.866025f;

InlineCurrentSense current_sense = InlineCurrentSense(0.002f, 50.0f, PA1, PA7, PB0);

float electrical_angle = 0.0;
float target_velocity = 10.0; // rad/s
unsigned long last_time = 0;

void configureTIM1() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER = (GPIOA->MODER & ~0x3F0000) | 0x2A0000; 
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF) | 0x666;
  GPIOB->MODER = (GPIOB->MODER & ~0xFC000000) | 0xA8000000; 
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000; 
  TIM1->CR1 = 0x20; TIM1->PSC = 0; TIM1->ARR = 4250;
  TIM1->CCMR1 = 0x6868; TIM1->CCMR2 = 0x68;
  TIM1->CCER = 0xFFF; // Active-Low Safe
  TIM1->BDTR = 0x8064; 
  TIM1->CR2 = 0xF3F; 
  TIM1->EGR |= 1; TIM1->CR1 |= 1; 
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC1EN);
  GPIOA->MODER |= (3 << 8) | (3 << 12); // PA4, PA6 Analog
  DAC1->CR |= 1; DAC1->DHR12R1 = 2048;

  configureTIM1();
  current_sense.init(); // This calibrates the zero-current offsets
  last_time = micros();
  Serial.println("Pure Voltage Mode - No PID, No Vibration.");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_time) * 1e-6f;
  if(dt <= 0 || dt > 0.01f) dt = 1e-4f;
  last_time = now;

  // 1. FIXED VOLTAGE (No PID = Smooth rotation)
  float Vq = 1.5f; // Push 1.5V into the motor
  float Vd = 0.0f;
  
  electrical_angle += target_velocity * dt;
  if (electrical_angle > _2PI) electrical_angle -= _2PI;

  float s = _sin(electrical_angle), c = _cos(electrical_angle);
  float V_alpha = Vd * c - Vq * s;
  float V_beta  = Vd * s + Vq * c;
  float Va = V_alpha, Vb = -0.5f * V_alpha + SQRT3_2 * V_beta, Vc = -0.5f * V_alpha - SQRT3_2 * V_beta;

  // Write to Silicon
  TIM1->CCR1 = (uint16_t)(_constrain((Va/POWER_SUPPLY)+0.5f, 0, 1) * 4250);
  TIM1->CCR2 = (uint16_t)(_constrain((Vb/POWER_SUPPLY)+0.5f, 0, 1) * 4250);
  TIM1->CCR3 = (uint16_t)(_constrain((Vc/POWER_SUPPLY)+0.5f, 0, 1) * 4250);

  // 2. READ THE TRUTH (Heavily filtered so you can see through the noise)
  static float filtered_Ia = 0, filtered_BEMF = 0;
  PhaseCurrent_s I = current_sense.getPhaseCurrents();
  
  // Software LPF to kill the PWM "scream"
  filtered_Ia = 0.99f * filtered_Ia + 0.01f * I.a;

  float vA = analogRead(BEMF_A), vB = analogRead(BEMF_B), vC = analogRead(BEMF_C);
  float zcA = vA - (vA + vB + vC)/3.0f;
  filtered_BEMF = 0.98f * filtered_BEMF + 0.02f * zcA;

  static unsigned long lp = 0;
  if (millis() - lp > 20) {
    // This will finally show you a steady current and a sine wave
    Serial.print("Ia_Filtered:"); Serial.print(filtered_Ia); Serial.print(",");
    Serial.print("BEMF_A:"); Serial.println(filtered_BEMF);
    lp = millis();
  }
}