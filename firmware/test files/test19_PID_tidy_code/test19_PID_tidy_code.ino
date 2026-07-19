// --- FOC Config for 2200KV ---
const int SINE_STEPS = 256;
int stepIndex = 0;
unsigned long previousMicros = 0;
int speedDelay = 200;        // SPEED UP: 2200KV needs higher RPM for stability

// --- FOC Variables ---
volatile float I_d = 0.0, I_q = 0.0;
volatile bool newDataReady = false;
float zeroU = 0, zeroV = 0;

// --- AGGRESSIVE PID SETTINGS ---
float target_Iq = 0.3;  
float target_Id = 0.0; 

// We are jacking these up so the Actual_Iq reaches the Target_Iq fast!
float Kp = 1.5; 
float Ki = 8.0;              // High Integral to kill the 0.06A -> 0.3A gap
float integral_q = 0.0, integral_d = 0.0;
float integral_limit = 800.0; // Prevent runaway vibration

float voltage_q = 0.0, voltage_d = 0.0;
float voltage_scale = 2500.0; // Increased "Punch"
float filter_alpha = 0.15;
float Iq_filtered = 0, Id_filtered = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // 1. OPAMP Setup (16x Gain)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  GPIOA->MODER |= (3 << (1 * 2)) | (3 << (7 * 2)); 
  GPIOB->MODER |= (3 << (0 * 2)); 
  uint32_t op_cfg = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9);
  OPAMP1->CSR = op_cfg; OPAMP2->CSR = op_cfg; OPAMP3->CSR = op_cfg;

  // 2. Calibration (Motor MUST be powered OFF)
  delay(1000); 
  long sU = 0, sV = 0;
  for(int i=0; i<100; i++){ sU += analogRead(PB1); sV += analogRead(PA2); delay(1); }
  zeroU = sU/100.0; zeroV = sV/100.0;

  // 3. Timer 1 (20kHz Center-Aligned)
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
  GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 
  TIM1->PSC = 0; TIM1->ARR = 8500 - 1;    
  TIM1->CCMR1 |= (6 << 4) | (6 << 12); TIM1->CCMR2 |= (6 << 4);
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  TIM1->BDTR |= TIM_BDTR_MOE;

  HardwareTimer *MyTim = new HardwareTimer(TIM1);
  MyTim->attachInterrupt(syncMeasurementISR);
  TIM1->DIER |= TIM_DIER_UIE; 
  TIM1->CR1 |= TIM_CR1_CEN;
}

void syncMeasurementISR() {
  static int div = 0;
  if (++div >= 100) { 
    div = 0;
    float u = (((analogRead(PB1) - zeroU) / 4095.0) * 3.3) / 0.16;
    float v = (((analogRead(PA2) - zeroV) / 4095.0) * 3.3) / 0.16;

    float angle = (stepIndex / (float)SINE_STEPS) * 2.0 * PI;
    float cosA = cos(angle), sinA = sin(angle);
    
    float alpha = u;
    float beta = 0.57735 * (u + 2.0 * v);
    I_d = alpha * cosA + beta * sinA;
    I_q = -alpha * sinA + beta * cosA;

    Id_filtered = (filter_alpha * I_d) + ((1.0 - filter_alpha) * Id_filtered);
    Iq_filtered = (filter_alpha * I_q) + ((1.0 - filter_alpha) * Iq_filtered);
    newDataReady = true;
  }
}

void loop() {
  if (newDataReady) {
    newDataReady = false;

    float eq = target_Iq - Iq_filtered;
    float ed = target_Id - Id_filtered;

    // Integral with Anti-Windup
    integral_q += eq * 0.01;
    if(integral_q > integral_limit) integral_q = integral_limit;
    if(integral_q < -integral_limit) integral_q = -integral_limit;

    voltage_q = (Kp * eq + Ki * integral_q) * voltage_scale; 
    voltage_d = (Kp * ed + Ki * integral_d) * voltage_scale;

    // Safety Saturation
    if (voltage_q > 3500) voltage_q = 3500;
    if (voltage_q < -3500) voltage_q = -3500;

    Serial.print("Tgt:"); Serial.print(target_Iq); Serial.print(",");
    Serial.print("Iq:"); Serial.print(Iq_filtered, 3); Serial.print(",");
    Serial.print("Id:"); Serial.println(Id_filtered, 3);
  }

  if (micros() - previousMicros >= speedDelay) {
    previousMicros = micros();
    float angle = (stepIndex / (float)SINE_STEPS) * 2.0 * PI;
    
    // Inverse Transforms (The Vector Steering)
    float V_alpha = voltage_d * cos(angle) - voltage_q * sin(angle);
    float V_beta  = voltage_d * sin(angle) + voltage_q * cos(angle);

    int offset = 4250; 
    TIM1->CCR1 = offset + (int)(V_alpha);
    TIM1->CCR2 = offset + (int)(-0.5 * V_alpha + 0.866 * V_beta);
    TIM1->CCR3 = offset + (int)(-0.5 * V_alpha - 0.866 * V_beta);

    if (++stepIndex >= SINE_STEPS) stepIndex = 0;
  }
}