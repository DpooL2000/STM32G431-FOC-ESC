// =================================================================
// BARE-METAL SIX-STEP COMMUTATOR - WEACT STM32G431
// V1.6 | EMI Bulletproofing | ADC Valley Protection | Failsafe
// =================================================================
#include <Arduino.h>
#include <SimpleFOC.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f

HardwareSerial ELRS_Serial(PB7, PB6); 
const int THROTTLE_CH = 2; 

#define DRIVER_OFF HIGH 

// --- MOTOR & SYSTEM CONFIG ---
const float POWER_SUPPLY = 12.0f;
const int   POLE_PAIRS   = 7;      
const float BEMF_MULTIPLIER = (3.3f / 4095.0f) * 5.70f;

// --- STATE VARIABLES ---
float elrs_throttle = 0.0f; 
bool  is_armed      = false;

// --- CLOSED LOOP STATE MACHINE ---
bool     closed_loop = false;
int      current_sector = 1;
uint32_t last_commutation_us = 0;
uint32_t zc_time_us = 0;
uint32_t step_delay_us = 4000; 
bool     zc_detected = false;

// --- STALL PROTECTION ---
bool     is_stalled = false;
uint32_t stall_timer_ms = 0;

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
// 4. SIX-STEP COMMUTATOR
// ==========================================
void setCommutation(int sector, float duty) {
  uint16_t ccr_val = (uint16_t)(duty * 4250.0f);
  uint32_t base_ccer = TIM_CCER_CC1P | TIM_CCER_CC1NP | 
                       TIM_CCER_CC2P | TIM_CCER_CC2NP | 
                       TIM_CCER_CC3P | TIM_CCER_CC3NP;

  if (sector == 0) {
    TIM1->CCER = base_ccer; 
    return;
  }

  TIM1->CCER = base_ccer; 

  switch(sector) {
    case 1: TIM1->CCR1 = ccr_val; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCER = base_ccer | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE; break;
    case 2: TIM1->CCR1 = ccr_val; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCER = base_ccer | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE; break;
    case 3: TIM1->CCR1 = 0; TIM1->CCR2 = ccr_val; TIM1->CCR3 = 0; TIM1->CCER = base_ccer | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE; break;
    case 4: TIM1->CCR1 = 0; TIM1->CCR2 = ccr_val; TIM1->CCR3 = 0; TIM1->CCER = base_ccer | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE; break;
    case 5: TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = ccr_val; TIM1->CCER = base_ccer | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE; break;
    case 6: TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = ccr_val; TIM1->CCER = base_ccer | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE; break;
  }
}

// ==========================================
// 5. BLDC AUDIO TONE ENGINE
// ==========================================
void playTone(int freq_hz, int duration_ms) {
  uint32_t period_us = 1000000 / freq_hz;
  uint32_t half_period = period_us / 2;
  uint32_t start_time = millis();
  
  uint16_t beep_ccr = (uint16_t)(0.20f * 4250.0f); 
  uint32_t base_ccer = TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP | TIM_CCER_CC3P | TIM_CCER_CC3NP;

  while (millis() - start_time < duration_ms) {
    TIM1->CCR1 = beep_ccr; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
    TIM1->CCER = base_ccer | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    delayMicroseconds(half_period);
    
    TIM1->CCER = base_ccer;
    delayMicroseconds(half_period);
  }
  TIM1->CCER = base_ccer; 
}

// ==========================================
// 6. ELRS CRSF PROTOCOL PARSER & FAILSAFE
// ==========================================
void readELRS() {
  static uint8_t crsf_buf[64];
  static uint8_t crsf_idx = 0;
  static uint32_t last_packet_ms = 0;
  
  // 1. BULLETPROOFING: Clear hardware UART errors caused by EMI stall spikes
  if (USART1->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
      USART1->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
  }
  
  while (ELRS_Serial.available()) {
    uint8_t b = ELRS_Serial.read();
    if (crsf_idx == 0 && b != 0xC8) continue; 
    crsf_buf[crsf_idx++] = b;
    
    if (crsf_idx >= 2 && crsf_idx >= crsf_buf[1] + 2) { 
        if (crsf_buf[2] == 0x16) { 
          last_packet_ms = millis(); // Packet received!
          
          uint16_t ch[4];
          ch[0] = (crsf_buf[3] | crsf_buf[4] << 8) & 0x07FF;
          ch[1] = (crsf_buf[4] >> 3 | crsf_buf[5] << 5) & 0x07FF;
          ch[2] = (crsf_buf[5] >> 6 | crsf_buf[6] << 2 | crsf_buf[7] << 10) & 0x07FF;
          ch[3] = (crsf_buf[7] >> 1 | crsf_buf[8] << 7) & 0x07FF;

          float raw_thr = (ch[THROTTLE_CH] - 172) / (1811.0f - 172.0f);
          if (raw_thr < 0.05f) raw_thr = 0.0f; 
          if (raw_thr > 1.0f)  raw_thr = 1.0f;
          elrs_throttle = raw_thr;
          
          if (!is_armed && elrs_throttle == 0.0f) {
             is_armed = true;
             Serial.println("ARMING DETECTED! Playing Doo-Wop...");
             playTone(880,  250); 
             delay(50);
             playTone(1175, 400); 
          }
        }
        crsf_idx = 0; 
    }
    if (crsf_idx >= 64) crsf_idx = 0; 
  }
  
  // 2. FAILSAFE: If radio disconnects or UART freezes, kill throttle instantly
  if (millis() - last_packet_ms > 200) {
      elrs_throttle = 0.0f;
  }
}

// ==========================================
// 7. SETUP
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(115200);
  delay(2000);

  Serial.println("--- SIX-STEP ESC KERNEL V1.6 ---");

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  setup_bare_metal_ADCs();
  configureTIM1();  

  ELRS_Serial.setRx(PB7);
  ELRS_Serial.setTx(PB6);
  ELRS_Serial.begin(420000);

  Serial.println("Powering Up... Playing Init Tones");
  playTone(1175, 150); 
  playTone(1319, 150); 
  playTone(1397, 150); 
  delay(400);

  int cell_count = (int)(POWER_SUPPLY / 3.7f);
  Serial.print("Cell Count Detected: "); Serial.println(cell_count);
  for (int i = 0; i < cell_count; i++) {
    playTone(1976, 150); 
    delay(100);
  }
  delay(100);
  
  last_commutation_us = micros();
}

// ==========================================
// 8. MAIN LOOP
// ==========================================
void loop() {
  uint32_t now_us = micros();
  static uint32_t last_loop_us = 0;
  static float current_active_voltage = 0.0f; 
  static int sync_error_count = 0; 
  
  float dt = (now_us - last_loop_us) * 1e-6f;
  if (dt <= 0.0f || dt > 0.1f) dt = 1e-4f; 
  last_loop_us = now_us;

  readELRS();

  // --- STALL RECOVERY COOLDOWN ---
  if (is_stalled) {
    if (millis() - stall_timer_ms > 2000) { 
      is_stalled = false;                 
      last_commutation_us = now_us;     
      current_active_voltage = 0.0f;      
      step_delay_us = 4000;               
      sync_error_count = 0; 
      Serial.println("Attempting Clean Restart... (Forced Open-Loop Ramping)");
    } else {
      setCommutation(0, 0.0f);            
      return;
    }
  }

  if (!is_stalled) {
    float volts_U = ADC1->JDR1 * BEMF_MULTIPLIER;
    float volts_V = ADC1->JDR2 * BEMF_MULTIPLIER;
    float volts_W = ADC1->JDR3 * BEMF_MULTIPLIER;
    float neutral = (volts_U + volts_V + volts_W) / 3.0f;

    float floating_bemf = 0.0f;
    if (current_sector == 3 || current_sector == 6) floating_bemf = volts_U - neutral; 
    else if (current_sector == 2 || current_sector == 5) floating_bemf = volts_V - neutral; 
    else if (current_sector == 1 || current_sector == 4) floating_bemf = volts_W - neutral; 

    // =======================================================
    // COMMUTATION ENGINE (V1.6.1 - Phantom Strike Fix)
    // =======================================================
    if (is_armed && elrs_throttle > 0.0f) {
      
      float target_voltage = 0.0f;
      float requested_voltage = constrain(1.5f + (elrs_throttle * (POWER_SUPPLY - 1.5f)), 0.0f, POWER_SUPPLY);

      if (step_delay_us < 1000 && (millis() - stall_timer_ms > 2000)) {
         closed_loop = true;
      } else {
         closed_loop = false;
      }

      // 1. V/f CURVE OVERRIDE
      if (!closed_loop) {
        float spool_progress = (4000.0f - (float)step_delay_us) / 3000.0f;
        float safe_spool_voltage = 1.5f + (spool_progress * 2.5f); 
        target_voltage = min(safe_spool_voltage, requested_voltage);
      } else {
        target_voltage = requested_voltage;
      }

      // 2. SLEW RATE LIMITER
      if (current_active_voltage < target_voltage) {
        current_active_voltage += 120.0f * dt; 
        if (current_active_voltage > target_voltage) current_active_voltage = target_voltage;
      } else {
        current_active_voltage = target_voltage;
      }

      // 3. DUTY CYCLE CLAMP (Reserve 5% for Valley Sampling)
      float duty_cycle = current_active_voltage / POWER_SUPPLY;
      if (duty_cycle > 0.95f) duty_cycle = 0.95f; 

      uint32_t time_since_commutation = now_us - last_commutation_us;

      // 4. PREVENT MATH IMPLOSION
      if (step_delay_us < 25) step_delay_us = 25; 

      // 5. SMART AUTO-ADVANCE WATCHDOG
      uint32_t dynamic_timeout = (closed_loop) ? (step_delay_us * 3) : 6000;
      if (closed_loop && dynamic_timeout < 1500) dynamic_timeout = 1500; // Restore grace period
      
      if (time_since_commutation > dynamic_timeout) {
          sync_error_count++; 
          
          if (sync_error_count > 6) { // Forgiving 6-strike tolerance
             setCommutation(0, 0.0f);        
             is_stalled = true;              
             stall_timer_ms = millis();      
             closed_loop = false;            
             step_delay_us = 4000;           
             zc_detected = false;  
             current_active_voltage = 0.0f;           
             Serial.println("TRUE STALL! Prop jammed. Coasting for 2s...");
             return;
          } else {
             // Force auto-advance to keep momentum alive
             current_sector++;
             if (current_sector > 6) current_sector = 1;
             setCommutation(current_sector, duty_cycle);
             last_commutation_us = now_us;
             zc_detected = false;
          }
      } else {
        // --- NORMAL BEMF TRACKING ---
        if (closed_loop) {
          if (!zc_detected) {
            if (time_since_commutation > (step_delay_us / 2)) {
              bool expected_rising = (current_sector == 2 || current_sector == 4 || current_sector == 6);
              
              if ((expected_rising && floating_bemf > 0.2f) || (!expected_rising && floating_bemf < -0.2f)) {
                zc_time_us = now_us;
                uint32_t raw_step_time = zc_time_us - last_commutation_us;
                step_delay_us = (step_delay_us * 8 + raw_step_time * 2) / 10; 
                zc_detected = true;
              }
            }
          } else {
            if ((now_us - zc_time_us) >= step_delay_us) {
              current_sector++;
              if (current_sector > 6) current_sector = 1;
              
              setCommutation(current_sector, duty_cycle);
              last_commutation_us = now_us;
              zc_detected = false;
              sync_error_count = 0; // SUCCESS! Clear all false-positive strikes.
            }
          }
        } 
        else {
          if (time_since_commutation >= step_delay_us) {
            current_sector++;
            if (current_sector > 6) current_sector = 1;
            
            setCommutation(current_sector, duty_cycle);
            last_commutation_us = now_us;
            sync_error_count = 0; // PREVENT PHANTOM STRIKES IN OPEN LOOP
            
            if (step_delay_us > 950) step_delay_us -= 10; 
          }
        }
      }
    } else {
      setCommutation(0, 0.0f);
      closed_loop = false;
      step_delay_us = 4000; 
      current_active_voltage = 0.0f; 
      sync_error_count = 0;
      last_commutation_us = now_us;  
    }
  }
}