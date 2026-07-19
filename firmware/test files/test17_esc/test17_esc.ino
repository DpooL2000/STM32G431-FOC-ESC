// =================================================================
// BARE-METAL SIX-STEP COMMUTATOR - WEACT STM32G431
// Active-Low 6N137 Logic | ELRS CRSF | V/f Spooling
// =================================================================
#include <Arduino.h>
#include <SimpleFOC.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f

// Map USART1 to PB7 (RX) and PB6 (TX)
HardwareSerial ELRS_Serial(PB7, PB6); 

#define DRIVER_OFF HIGH 

// --- MOTOR & SYSTEM CONFIG ---
const float POWER_SUPPLY = 12.0f;
const int   POLE_PAIRS   = 7;      // Standard for 5-inch drone motors (12N14P)
const float BEMF_MULTIPLIER = (3.3f / 4095.0f) * 5.70f;

// --- STATE VARIABLES ---
float target_velocity  = 0.0f; 
float electrical_angle = 0.0f;
float elrs_throttle    = 0.0f; 

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

// ==========================================
// 1. HARDWARE LOCKDOWN
// ==========================================
void hardware_lockdown() {
  pinMode(PA8,  OUTPUT); pinMode(PA9,  OUTPUT); pinMode(PA10, OUTPUT);
  pinMode(PB13, OUTPUT); pinMode(PB14, OUTPUT); pinMode(PB15, OUTPUT);
  digitalWrite(PA8,  DRIVER_OFF); digitalWrite(PB13, DRIVER_OFF);
  digitalWrite(PA9,  DRIVER_OFF); digitalWrite(PB14, DRIVER_OFF);
  digitalWrite(PA10, DRIVER_OFF); digitalWrite(PB15, DRIVER_OFF);
}

// ==========================================
// 2. TIM1 (CENTER-ALIGNED PWM & TRGO)
// ==========================================
void configureTIM1() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk);
  GPIOA->MODER |=  (2 << GPIO_MODER_MODE8_Pos) | (2 << GPIO_MODER_MODE9_Pos) | (2 << GPIO_MODER_MODE10_Pos);
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0x00000FFF) | (6<<0) | (6<<4) | (6<<8);

  GPIOB->MODER &= ~(GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk);
  GPIOB->MODER |=  (2 << GPIO_MODER_MODE13_Pos) | (2 << GPIO_MODER_MODE14_Pos) | (2 << GPIO_MODER_MODE15_Pos);
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | (6<<20) | (6<<24) | (4<<28);

  TIM1->CR1   = TIM_CR1_CMS_0; 
  TIM1->PSC   = 0;
  TIM1->ARR   = 4250;
  
  TIM1->CCMR1 = (6<<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6<<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = (6<<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

  // ACTIVE-LOW POLARITY FOR 6N137
  TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC1P | TIM_CCER_CC1NP |
                TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC2P | TIM_CCER_CC2NP |
                TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3P | TIM_CCER_CC3NP;

  TIM1->CR2   = TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_OIS2 | TIM_CR2_OIS2N | TIM_CR2_OIS3 | TIM_CR2_OIS3N;
  TIM1->CR2  &= ~TIM_CR2_MMS_Msk; 
  TIM1->CR2  |= (0x2 << TIM_CR2_MMS_Pos); 
  
  TIM1->BDTR  = TIM_BDTR_MOE | (100 << TIM_BDTR_DTG_Pos);
  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0; 
  TIM1->EGR  = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
}

// ==========================================
// 3. ADC1 (INJECTED BEMF SCAN)
// ==========================================
void setup_bare_metal_ADCs() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

  GPIOA->MODER |= (3 << GPIO_MODER_MODE0_Pos) | (3 << GPIO_MODER_MODE3_Pos);
  GPIOB->MODER |= (3 << GPIO_MODER_MODE1_Pos);

  GPIOA->MODER |= (3 << GPIO_MODER_MODE4_Pos) | (3 << GPIO_MODER_MODE6_Pos);
  RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
  DAC1->CR     |= DAC_CR_EN1 | DAC_CR_EN2;
  DAC1->DHR12R1 = 2048;
  DAC1->DHR12R2 = 2048;

  RCC->AHB2ENR      |= RCC_AHB2ENR_ADC12EN;
  ADC12_COMMON->CCR &= ~(3 << 16);
  ADC12_COMMON->CCR |=  (1 << 16);

  ADC1->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |=  ADC_CR_ADVREGEN; delay(1);
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL); delay(1);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));

  ADC1->SMPR1 |= (3 << 3) | (3 << 12);  
  ADC1->SMPR2 |= (3 << 6);              

  ADC1->JSQR = (2 << 0) | (1 << 7) | (0 << 2) | (1 << 9) | (4 << 15) | (12 << 21);
  ADC1->CR |= ADC_CR_JADSTART;
}

// ==========================================
// 4. SIX-STEP COMMUTATOR LOGIC
// ==========================================
void setCommutation(int sector, float duty) {
  uint16_t ccr_val = (uint16_t)(duty * 4250.0f);
  uint32_t base_ccer = TIM_CCER_CC1P | TIM_CCER_CC1NP | 
                       TIM_CCER_CC2P | TIM_CCER_CC2NP | 
                       TIM_CCER_CC3P | TIM_CCER_CC3NP;

  TIM1->CCER = base_ccer; 

  switch(sector) {
    case 1: 
      TIM1->CCR1 = ccr_val; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
      TIM1->CCER = base_ccer | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
      break;
    case 2: 
      TIM1->CCR1 = ccr_val; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
      TIM1->CCER = base_ccer | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      break;
    case 3: 
      TIM1->CCR1 = 0; TIM1->CCR2 = ccr_val; TIM1->CCR3 = 0;
      TIM1->CCER = base_ccer | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      break;
    case 4: 
      TIM1->CCR1 = 0; TIM1->CCR2 = ccr_val; TIM1->CCR3 = 0;
      TIM1->CCER = base_ccer | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
      break;
    case 5: 
      TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = ccr_val;
      TIM1->CCER = base_ccer | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      break;
    case 6: 
      TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = ccr_val;
      TIM1->CCER = base_ccer | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      break;
  }
}

// ==========================================
// 5. ELRS CRSF PROTOCOL PARSER
// ==========================================
void readELRS() {
  static uint8_t crsf_buf[64];
  static uint8_t crsf_idx = 0;
  
  while (ELRS_Serial.available()) {
    uint8_t b = ELRS_Serial.read();
    
    if (crsf_idx == 0 && b != 0xC8) continue; 
    
    crsf_buf[crsf_idx++] = b;
    
    if (crsf_idx >= 2) {
      uint8_t len = crsf_buf[1];
      if (crsf_idx >= len + 2) { 
        
        if (crsf_buf[2] == 0x16) { 
          // Decode first 4 channels
          uint16_t ch0 = (crsf_buf[3] | crsf_buf[4] << 8) & 0x07FF;
          uint16_t ch1 = (crsf_buf[4] >> 3 | crsf_buf[5] << 5) & 0x07FF;
          uint16_t ch2 = (crsf_buf[5] >> 6 | crsf_buf[6] << 2 | crsf_buf[7] << 10) & 0x07FF;
          uint16_t ch3 = (crsf_buf[7] >> 1 | crsf_buf[8] << 7) & 0x07FF;

          // AETR Mapping: Throttle is usually Channel 3 (index 2)
          float raw_thr = (ch2 - 172) / (1811.0f - 172.0f);
          
          if (raw_thr < 0.05f) raw_thr = 0.0f; 
          if (raw_thr > 1.0f)  raw_thr = 1.0f;
          
          elrs_throttle = raw_thr;
          target_velocity = elrs_throttle * 3000.0f; // Max open-loop target
        }
        crsf_idx = 0; 
      }
    }
    if (crsf_idx >= 64) crsf_idx = 0; 
  }
}

// ==========================================
// 6. SETUP
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(115200);
  delay(2000);

  Serial.println("--- SIX-STEP ESC WITH ELRS ---");

  // Enable DWT Cycle Counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  setup_bare_metal_ADCs();
  configureTIM1();  

  command.add('T', doTarget, "target velocity");
  
  ELRS_Serial.begin(420000);
}

// ==========================================
// 7. MAIN LOOP
// ==========================================
void loop() {
  // 1. PRECISE SILICON TIMING
  static uint32_t last_cycles = 0;
  uint32_t now_cycles = DWT->CYCCNT;
  
  // 170MHz Clock -> seconds
  float dt = (float)(now_cycles - last_cycles) * (1.0f / 170000000.0f);
  if (dt <= 0.0f || dt > 0.01f) dt = 1e-4f;
  last_cycles = now_cycles;

  // 2. PARSE RADIO
  readELRS();

  // 3. OPEN-LOOP ANGLE ADVANCEMENT
  electrical_angle += target_velocity * dt;
  if (electrical_angle >= _2PI) electrical_angle -= _2PI;
  if (electrical_angle < 0.0f)  electrical_angle += _2PI;

  int current_sector = (int)(electrical_angle / (_PI / 3.0f)) % 6 + 1;

  // 4. V/f CURVE: Voltage MUST scale with speed to overcome BEMF
  // Minimum push of 1.5V, scales up to 5V at max speed.
  float active_voltage = 0.0f;
  if (target_velocity > 0.0f) {
    active_voltage = constrain(1.5f + (elrs_throttle * 3.5f), 0.0f, POWER_SUPPLY);
  }
  
  float duty_cycle = active_voltage / POWER_SUPPLY;
  setCommutation(current_sector, duty_cycle);

  // 5. READ SYNCHRONIZED ADCs 
  float volts_A = ADC1->JDR1 * BEMF_MULTIPLIER;
  float volts_B = ADC1->JDR2 * BEMF_MULTIPLIER;
  float volts_C = ADC1->JDR3 * BEMF_MULTIPLIER;
  float neutral = (volts_A + volts_B + volts_C) / 3.0f;

  float floating_bemf = 0.0f;
  if (current_sector == 3 || current_sector == 6) floating_bemf = volts_A - neutral; 
  else if (current_sector == 2 || current_sector == 5) floating_bemf = volts_B - neutral; 
  else if (current_sector == 1 || current_sector == 4) floating_bemf = volts_C - neutral; 

  // 6. TELEMETRY (10 Hz)
  static uint32_t last_print_ms = 0;
  if (millis() - last_print_ms > 100) { 
    last_print_ms = millis();
    
    // Calculate Mechanical RPM (rad/s to RPM)
    float mech_rpm = (target_velocity / POLE_PAIRS) * 9.549297f; 
    
    // Calculate Loop Time in microseconds
    float loop_us = (float)(DWT->CYCCNT - now_cycles) * (1000000.0f / 170000000.0f);

    Serial.print("Thr%:");   Serial.print(elrs_throttle * 100.0f, 0); 
    Serial.print(" | eSpd(rad/s):"); Serial.print(target_velocity, 0); 
    Serial.print(" | mRPM:"); Serial.print(mech_rpm, 0); 
    Serial.print(" | Vout:"); Serial.print(active_voltage, 2); 
    Serial.print(" | Loop(us):"); Serial.println(loop_us, 2);
  }

  command.run();
}