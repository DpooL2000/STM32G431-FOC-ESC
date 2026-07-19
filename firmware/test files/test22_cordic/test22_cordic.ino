/**
 * STM32G431 - High-Speed FOC with Alignment Phase
 */

#include <Arduino.h>

const int SINE_STEPS = 256;
int stepIndex = 0;
unsigned long previousMicros = 0;

// --- STATE MACHINE ---
enum MotorState { ALIGNING, RAMPING };
MotorState currentState = ALIGNING;
unsigned long alignStartTime = 0;

// --- RAMP & ADVANCE TUNING (1000KV) ---
float currentSpeedDelay = 2000.0; // Start slow
float targetSpeedDelay = 25.0;    // Target speed
float acceleration = 0.2;        

float phase_advance_factor = 0.0001; 

// --- VOLTAGE TUNING (1000KV) ---
float voltage_align = 1200.0; // Voltage to "snap" the rotor into place
float voltage_q = 1500.0;     // Run voltage
float voltage_d = 0.0;

volatile float Iq_filtered = 0, Id_filtered = 0;
volatile bool newDataReady = false;
float zeroU = 0, zeroV = 0;

// --- CORRECTED CORDIC SETUP ---
void initCORDIC() {
  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN; 
  CORDIC->CSR = (1 << 20) | (0 << 16) | (5 << 4); 
}

void fastSinCos(float angle_rad, float &out_sin, float &out_cos) {
  while(angle_rad > PI) angle_rad -= 2.0 * PI;
  while(angle_rad < -PI) angle_rad += 2.0 * PI;

  int32_t q31_angle = (int32_t)((angle_rad / PI) * 2147483647.0f);
  CORDIC->WDATA = q31_angle;

  out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
  out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  initCORDIC(); 

  // 1. Hardware Init (OPAMPs)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  GPIOA->MODER |= (3 << (1 * 2)) | (3 << (7 * 2)); 
  GPIOB->MODER |= (3 << (0 * 2)); 
  uint32_t op_cfg = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9);
  OPAMP1->CSR = op_cfg; OPAMP2->CSR = op_cfg; OPAMP3->CSR = op_cfg;

  // 2. Calibration
  delay(500); 
  long sU = 0, sV = 0;
  for(int i=0; i<100; i++){ sU += analogRead(PB1); sV += analogRead(PA2); delay(1); }
  zeroU = sU/100.0; zeroV = sV/100.0;

  // 3. Timer 1 (20kHz PWM)
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
  GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 
  TIM1->PSC = 0; TIM1->ARR = 8500 - 1;    
  TIM1->CR1 |= TIM_CR1_CMS_0; 
  TIM1->CCMR1 |= (6 << 4) | (6 << 12); TIM1->CCMR2 |= (6 << 4);
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  TIM1->BDTR |= TIM_BDTR_MOE;

  HardwareTimer *MyTim = new HardwareTimer(TIM1);
  MyTim->attachInterrupt(syncMeasurementISR);
  TIM1->DIER |= TIM_DIER_UIE; 
  TIM1->CR1 |= TIM_CR1_CEN;

  alignStartTime = millis(); // Start the alignment clock
}

void syncMeasurementISR() {
  static int div = 0;
  if (++div >= 20) { 
    div = 0;
    float u = (((analogRead(PB1) - zeroU) / 4095.0) * 3.3) / 0.16;
    float v = (((analogRead(PA2) - zeroV) / 4095.0) * 3.3) / 0.16;

    float angle = (stepIndex / (float)SINE_STEPS) * 2.0 * PI;
    float cosA, sinA;
    fastSinCos(angle, sinA, cosA); 
    
    float alpha = u, beta = 0.57735 * (u + 2.0 * v);
    float rIq = -alpha * sinA + beta * cosA;
    float rId = alpha * cosA + beta * sinA;
    
    Iq_filtered = (0.1 * rIq) + (0.9 * Iq_filtered);
    Id_filtered = (0.1 * rId) + (0.9 * Id_filtered);
    newDataReady = true;
  }
}

void loop() {
  unsigned long currentMicros = micros();

  // --- PHASE 1: ALIGNMENT ---
  if (currentState == ALIGNING) {
    if (millis() - alignStartTime < 1500) { // Hold for 1.5 seconds
      // Force angle to 0. Force V_d to alignment voltage.
      float V_alpha = voltage_align; 
      float V_beta  = 0.0;

      TIM1->CCR1 = 4250 + (int)(V_alpha);
      TIM1->CCR2 = 4250 + (int)(-0.5 * V_alpha);
      TIM1->CCR3 = 4250 + (int)(-0.5 * V_alpha);
      
      stepIndex = 0; // Keep the angle frozen at 0
    } else {
      Serial.println("Alignment Complete. Ramping...");
      currentState = RAMPING;
      previousMicros = micros();
    }
  } 
  
  // --- PHASE 2: RAMPING ---
  else if (currentState == RAMPING) {
    if (currentMicros - previousMicros >= (unsigned long)currentSpeedDelay) {
      previousMicros = currentMicros;

      if (currentSpeedDelay > targetSpeedDelay) {
        currentSpeedDelay -= acceleration; 
      }

      float speed_rpm = 60000000.0 / (currentSpeedDelay * SINE_STEPS * 7);
      float advance_angle = speed_rpm * phase_advance_factor; 

      float base_angle = (stepIndex / (float)SINE_STEPS) * 2.0 * PI;
      float final_angle = base_angle + advance_angle;

      float V_sin, V_cos;
      fastSinCos(final_angle, V_sin, V_cos);

      float V_alpha = -voltage_q * V_sin;
      float V_beta  =  voltage_q * V_cos;

      TIM1->CCR1 = 4250 + (int)(V_alpha);
      TIM1->CCR2 = 4250 + (int)(-0.5 * V_alpha + 0.866 * V_beta);
      TIM1->CCR3 = 4250 + (int)(-0.5 * V_alpha - 0.866 * V_beta);

      if (++stepIndex >= SINE_STEPS) stepIndex = 0;
    }
  }

  // Telemetry 
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 100) {
  //   lastPrint = millis();
  //   if (newDataReady && currentState == RAMPING) {
  //     newDataReady = false;
  //     Serial.print("RPM:"); Serial.print(60000000.0 / (currentSpeedDelay * SINE_STEPS * 7));
  //     Serial.print(", Iq:"); Serial.print(Iq_filtered, 3);
  //     Serial.print(", Id:"); Serial.println(Id_filtered, 3);
  //   }
  // }
}