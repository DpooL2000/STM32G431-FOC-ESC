// --- SPWM Open-Loop Variables ---
const int SINE_STEPS = 256;
int sineTable[SINE_STEPS];
int stepIndex = 0;
unsigned long previousMicros = 0;
int speedDelay = 500; // 3ms per step (Slow, safe, won't stall)

// --- FOC Measurement Variables ---
volatile float I_alpha = 0.0;
volatile float I_beta  = 0.0;
volatile float I_d     = 0.0;
volatile float I_q     = 0.0;
volatile bool newDataReady = false;

// Hardware Calibration Variables
float zeroU = 0;
float zeroV = 0;

// --- NEW PID VARIABLES ---
float target_Iq = 0.3; // Target Torque in Amps (Start low and safe!)
float target_Id = 0.0; // We always want 0 wasted flux

float Kp = 10.0;  // Proportional Gain
float Ki = 1000;  // Integral Gain
float integral_q = 0.0;
float integral_d = 0.0;

float voltage_q = 0.0;
float voltage_d = 0.0;

// --- ADD THIS TO YOUR GLOBALS ---
float voltage_scale = 2000.0; // Scales 1.0 Amp error to 2000 PWM units
float filter_alpha = 0.2; // Smoothing factor (0.1 = heavy filter, 1.0 = no filter)
float Iq_filtered = 0;
float Id_filtered = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // --- 1. GENERATE THE SAFE SINE TABLE ---
  int safePWM = 1300; // ~15% power to break cogging torque smoothly
  int amplitude = safePWM / 2;
  int offset = safePWM / 2; 
  
  for(int i = 0; i < SINE_STEPS; i++) {
    float angle = (i / (float)SINE_STEPS) * 2.0 * PI;
    sineTable[i] = offset + (int)(amplitude * sin(angle));
  }

  // --- 2. BARE METAL OPAMP SETUP ---
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

  GPIOA->MODER |= (3 << (1 * 2)) | (3 << (7 * 2)); 
  GPIOB->MODER |= (3 << (0 * 2)); 

  // 16x Gain (Shifted by 5 for STM32G4)
  OPAMP1->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);
  OPAMP2->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);
  OPAMP3->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);

  // --- 3. AUTO-CALIBRATION ROUTINE ---
  // Wait for analog signals to stabilize
  delay(500); 
  long sumU = 0;
  long sumV = 0;
  
  // Read 100 times to find the exact hardware bias (the 47k + 1k divider)
  for(int i = 0; i < 100; i++) {
    sumU += analogRead(PB1);
    sumV += analogRead(PA2);
    delay(2);
  }
  
  zeroU = sumU / 100.0;
  zeroV = sumV / 100.0;
  
  // Print calibration values just so you know what they are
  Serial.print("Calibrated Zero U: "); Serial.println(zeroU);
  Serial.print("Calibrated Zero V: "); Serial.println(zeroV);
  delay(1000); // Give you a second to read it before motor starts

  // --- 4. BARE METAL TIMER 1 SETUP (20kHz) ---
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

  // --- 5. HARDWARE INTERRUPT (ADC SYNC) ---
  HardwareTimer *MyTim = new HardwareTimer(TIM1);
  MyTim->attachInterrupt(syncMeasurementISR);
  TIM1->DIER |= TIM_DIER_UIE; 

  TIM1->CR1 |= TIM_CR1_CEN;
}

void syncMeasurementISR() {
  static int div = 0;
  if (++div >= 100) { 
    div = 0;
    
    // 1. Current Sensing
    float u = (((analogRead(PB1) - zeroU) / 4095.0) * 3.3) / 0.16;
    float v = (((analogRead(PA2) - zeroV) / 4095.0) * 3.3) / 0.16;

    // 2. Clarke & Park
    float alpha = u;
    float beta = 0.57735 * (u + 2.0 * v);
    float angle = (stepIndex / (float)SINE_STEPS) * 2.0 * PI;
    float cosA = cos(angle);
    float sinA = sin(angle);

    float raw_Id = alpha * cosA + beta * sinA;
    float raw_Iq = -alpha * sinA + beta * cosA;

    // 3. LOW PASS FILTER (The "Stabilizer")
    Id_filtered = (filter_alpha * raw_Id) + ((1.0 - filter_alpha) * Id_filtered);
    Iq_filtered = (filter_alpha * raw_Iq) + ((1.0 - filter_alpha) * Iq_filtered);
    
    newDataReady = true;
  }
}

void loop() {
  if (newDataReady) {
    newDataReady = false;

    // PI CONTROLLER (Lowered Kp for stability)
    float eq = target_Iq - Iq_filtered;
    float ed = target_Id - Id_filtered;

    integral_q += eq * 0.01;
    integral_d += ed * 0.01;

    // Output is scaled to PWM units (0-4250)
    voltage_q = (0.8 * eq + 0.2 * integral_q) * 2000.0; 
    voltage_d = (0.8 * ed + 0.2 * integral_d) * 2000.0;

    // Saturation Limit
    if (voltage_q > 3000) voltage_q = 3000;
    if (voltage_q < -3000) voltage_q = -3000;

    Serial.print("Target_Iq:"); Serial.print(target_Iq); Serial.print(",");
    Serial.print("Actual_Iq:"); Serial.print(Iq_filtered, 4); Serial.print(",");
    Serial.print("Actual_Id:"); Serial.println(Id_filtered, 4);
  }

  // --- THE VECTOR OUTPUT (True Inverse FOC) ---
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= speedDelay) {
    previousMicros = currentMicros;

    float angle = (stepIndex / (float)SINE_STEPS) * 2.0 * PI;
    float cosA = cos(angle);
    float sinA = sin(angle);

    // Inverse Park: Convert d/q voltages back to stationary alpha/beta
    float V_alpha = voltage_d * cosA - voltage_q * sinA;
    float V_beta  = voltage_d * sinA + voltage_q * cosA;

    // Inverse Clarke: Convert alpha/beta to 3-phase PWM
    int offset = 4250; 
    TIM1->CCR1 = offset + (int)(V_alpha);
    TIM1->CCR2 = offset + (int)(-0.5 * V_alpha + 0.866 * V_beta);
    TIM1->CCR3 = offset + (int)(-0.5 * V_alpha - 0.866 * V_beta);

    if (++stepIndex >= 256) stepIndex = 0;
  }
}