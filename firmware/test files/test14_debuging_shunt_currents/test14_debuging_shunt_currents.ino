// =================================================================
// BARE-METAL AUTO-SPOOL CURRENT PLOTTER 
// Hardware-Triggered Injected Scan (G431 Edition)
// =================================================================
#include <Arduino.h>
#include <math.h>

#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

// --- STATE & CONTROL VARIABLES ---
float electrical_angle = 0.0;
float target_speed = 100.0f; // rad/s (Smooth, slow spin)
float amplitude = 0.0f;     // Starts at 0, ramps up safely

// --- ADC OFFSETS & FILTERS ---
int offset_U = 2048;
int offset_V = 2048;
int offset_W = 2048;

float filter_alpha = 0.05f; 
float filtered_U = 0.0, filtered_V = 0.0, filtered_W = 0.0;

// ==========================================
// 1. HARDWARE LOCKDOWN
// ==========================================
void hardware_lockdown() {
  pinMode(PA8,  OUTPUT); pinMode(PA9,  OUTPUT); pinMode(PA10, OUTPUT);
  pinMode(PB13, OUTPUT); pinMode(PB14, OUTPUT); pinMode(PB15, OUTPUT);
  digitalWrite(PA8, DRIVER_OFF);  digitalWrite(PB13, DRIVER_OFF); 
  digitalWrite(PA9, DRIVER_OFF);  digitalWrite(PB14, DRIVER_OFF); 
  digitalWrite(PA10, DRIVER_OFF); digitalWrite(PB15, DRIVER_OFF); 
}

// ==========================================
// 2. ADVANCED TIMER 1 (INVERTER PWM)
// ==========================================
void configureTIM1() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER = (GPIOA->MODER & ~0x3F0000) | 0x2A0000; 
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF) | 0x666;
  GPIOB->MODER = (GPIOB->MODER & ~0xFC000000) | 0xA8000000; 
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000; 
  
  TIM1->CR1 = 0x20; TIM1->PSC = 0; TIM1->ARR = 4250;
  TIM1->CCMR1 = 0x6868; TIM1->CCMR2 = 0x68;
  TIM1->CCER = 0xFFF;   
  TIM1->BDTR = 0x80A0;  
  
  TIM1->CR2 = 0xF3F; 
  // --- SYNC UPGRADE: Force TIM1 to generate a TRGO pulse on Update Event ---
  TIM1->CR2 &= ~TIM_CR2_MMS_Msk;             // Clear Master Mode bits
  TIM1->CR2 |= (0x2 << TIM_CR2_MMS_Pos);     // 010: Update Event = TRGO
  
  TIM1->EGR |= 1; TIM1->CR1 |= 1; 
}

// ==========================================
// 3. BARE-METAL ADCs (INJECTED HARDWARE SCAN)
// ==========================================
void setup_bare_metal_ADCs() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  GPIOA->MODER |= (3 << 2) | (3 << 8) | (3 << 12) | (3 << 14); 
  GPIOB->MODER |= (3 << 0);             
  
  RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;       
  DAC1->CR |= 1; DAC1->DHR12R1 = 2048;                     

  RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
  ADC12_COMMON->CCR &= ~(3 << 16); ADC12_COMMON->CCR |= (1 << 16);
  
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL); delay(1);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));
  
  // Clean Sample Times
  ADC1->SMPR1 |= (3 << 6); ADC1->SMPR2 |= (3 << 15); ADC2->SMPR1 |= (3 << 12);

  // --- INJECTED SEQUENCE SETUP ---
  // ADC1 JSQR: Length=2 (U then V), Trigger=Rising Edge, Source=TIM1_TRGO (0)
  // Channel 15 (PB0) -> JDR1
  // Channel 2 (PA1)  -> JDR2
  ADC1->JSQR = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15);
  
  // ADC2 JSQR: Length=1 (W), Trigger=Rising Edge, Source=TIM1_TRGO (0)
  // Channel 4 (PA7)  -> JDR1
  ADC2->JSQR = (0 << 0) | (1 << 7) | (4 << 9);


  // ==================================================
  // THE MISSING BITS: ARM THE HARDWARE TRIGGERS!
  // ==================================================
  ADC1->CR |= ADC_CR_JADSTART;
  ADC2->CR |= ADC_CR_JADSTART;
}

// ==========================================
// MAIN SETUP
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(115200);
  delay(3000); 

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
  DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  setup_bare_metal_ADCs();
  
  // --- CRITICAL REORDER ---
  // We MUST configure TIM1 before calibration. Since the ADCs are now strictly 
  // hardware-triggered, they won't collect data unless TIM1 is beating!
  configureTIM1(); 
  
  long sum_U = 0, sum_V = 0, sum_W = 0;
  for(int i = 0; i < 100; i++) {
      delay(2); // Wait 2ms (allows ~40 hardware triggers to happen in the background)
      sum_U += ADC1->JDR1; // U (PB0)
      sum_V += ADC1->JDR2; // V (PA1)
      sum_W += ADC2->JDR1; // W (PA7)
  }
  offset_U = sum_U / 100;
  offset_V = sum_V / 100;
  offset_W = sum_W / 100;

  Serial.print("RAW OFFSETS -> U: "); Serial.print(offset_U);
  Serial.print(" | V: "); Serial.print(offset_V);
  Serial.print(" | W: "); Serial.println(offset_W);

  Serial.println("System Ready. Auto-Spooling Motor with Injected Scan...");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop1() {
  uint32_t start_cycles = DWT->CYCCNT; 
  
  // 1. EXACT TIMING MATH
  static uint32_t last_cycles = 0;
  float dt = (float)(start_cycles - last_cycles) * 0.00000000588235f; 
  if(dt <= 0.0f || dt > 0.01f) dt = 1e-4f;
  last_cycles = start_cycles;

  // 2. AUTO RAMP-UP
  if (amplitude < 0.10f) {
      amplitude += 0.033f * dt; 
  }

  // 3. GENERATE SMOOTH SINE WAVES
  electrical_angle += target_speed * dt;
  while (electrical_angle >= _2PI) electrical_angle -= _2PI;
  while (electrical_angle < 0.0f) electrical_angle += _2PI;

  float duty_U = (sinf(electrical_angle) * amplitude) + 0.5f;
  float duty_V = (sinf(electrical_angle - 2.0944f) * amplitude) + 0.5f;
  float duty_W = (sinf(electrical_angle + 2.0944f) * amplitude) + 0.5f;

  TIM1->CCR1 = (uint16_t)(duty_U * 4250);
  TIM1->CCR2 = (uint16_t)(duty_V * 4250);
  TIM1->CCR3 = (uint16_t)(duty_W * 4250);

  // 4. GRAB INSTANT BACKGROUND HARDWARE DATA
  int raw_U = ADC1->JDR1; // PB0
  int raw_V = ADC1->JDR2; // PA1
  int raw_W = ADC2->JDR1; // PA7 (The suspect)

  // 5. PRINT RAW 12-BIT SILICON DATA TO PLOTTER
  static unsigned long last_plot_time = 0;
  if (millis() - last_plot_time > 20) { 
    Serial.print("Raw_U:"); Serial.print(raw_U); Serial.print(",");
    Serial.print("Raw_V:"); Serial.print(raw_V); Serial.print(",");
    Serial.print("Raw_W:"); Serial.println(raw_W);
    last_plot_time = millis();
  }
}

// =================================================================
// MAIN LOOP with Cycle Timing
// =================================================================
void loop() {
  // 1. Record Start Cycles
  uint32_t start_cycles = DWT->CYCCNT; 
  
  // 2. AUTO RAMP-UP
  // Use a fixed dt or calculated one if you have a timer. 
  // For now, using a small constant for demonstration if dt isn't defined.
  float dt = 0.000020f; // Assuming 20us loop based on your request
  if (amplitude < 0.10f) {
      amplitude += 0.033f * dt; 
  }

  // 3. GENERATE SMOOTH SINE WAVES
  electrical_angle += target_speed * dt;
  while (electrical_angle >= _2PI) electrical_angle -= _2PI;
  while (electrical_angle < 0.0f) electrical_angle += _2PI;

  float duty_U = (sinf(electrical_angle) * amplitude) + 0.5f;
  float duty_V = (sinf(electrical_angle - 2.0944f) * amplitude) + 0.5f;
  float duty_W = (sinf(electrical_angle + 2.0944f) * amplitude) + 0.5f;

  TIM1->CCR1 = (uint16_t)(duty_U * 4250);
  TIM1->CCR2 = (uint16_t)(duty_V * 4250);
  TIM1->CCR3 = (uint16_t)(duty_W * 4250);

  // 4. GRAB INSTANT BACKGROUND HARDWARE DATA
  int raw_U = ADC1->JDR1; // PB0
  int raw_V = ADC1->JDR2; // PA1
  int raw_W = ADC2->JDR1; // PA7

  // 5. CALCULATE CYCLES & LOOP TIME
  uint32_t end_cycles = DWT->CYCCNT;
  uint32_t cycles_taken = end_cycles - start_cycles;
  
  // 170MHz clock = 1 / 170,000,000 = 0.00000000588 seconds per cycle
  // Multiply cycles by 0.00588235 to get microseconds
  float loop_time_us = (float)cycles_taken * 0.00588235f; 

  // 6. TELEMETRY (Every 100ms to avoid Serial bottlenecking the loop)
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) { 
    // Serial.print("Cycles:"); Serial.print(cycles_taken);
    // Serial.print(" | Loop:"); Serial.print(loop_time_us, 2); Serial.println("us");
    
    // Keep Plotter data separate if you need it
    Serial.print("Raw_U:"); Serial.print(raw_U); Serial.print(",");
    Serial.print("Raw_V:"); Serial.print(raw_V); Serial.print(",");
    Serial.print("Raw_W:"); Serial.println(raw_W);
    
    last_print_time = millis();
  }
}