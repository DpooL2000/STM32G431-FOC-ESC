// =================================================================
// BARE-METAL FOC MASTER KERNEL V4.3 (PLL HANDOFF EDITION)
// Target: WeAct STM32G431 | 20kHz Hard-Sync | Binary Telemetry
// =================================================================
#include <Arduino.h>
#include <HardwareSerial.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

const float POWER_SUPPLY    = 12.0f;
const float LOOP_DT         = 50e-6f;
const float NOMINAL_AMPS_PER_COUNT = (3.3f / 4095.0f) / (0.002f * 20.0f);

// --- MOTOR DNA ---
const float RS     = 0.0930f;      
const float LS     = 0.0000468f;   
const float LAMBDA = 0.000300f;    

float offset_U = 2048.0f;
float offset_V = 2048.0f;

enum DriveState { STATE_OFF, STATE_ALIGN, STATE_SPOOL, STATE_CLOSED_LOOP_FOC };
DriveState current_state = STATE_OFF;

float cmd_v_alpha = 0.0f, cmd_v_beta = 0.0f, cmd_vq_throttle = 0.0f; 
float flux_alpha = 0.0f, flux_beta = 0.0f;
float estimated_angle = 0.0f, estimated_speed = 0.0f;        

HardwareSerial ELRS_Serial(PB7, PB6);
const int THROTTLE_CH = 2; 
bool is_armed = false;
float elrs_throttle = 0.0f;

// --- BINARY TELEMETRY STRUCT ---
struct TelemetryData {
  uint32_t sync = 0xDDBBCCAA;
  uint32_t loop_cycles;
  float    i_alpha;
  float    i_beta;
  float    v_alpha;
  float    v_beta;
  float    flux_alpha;
  float    flux_beta;
  float    est_angle;
  float    comm_angle; // The smoothed PLL angle
  uint8_t  state;
} __attribute__((packed));

TelemetryData t_data;

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
  TIM1->CCMR1 = 0x6868; TIM1->CCMR2 = 0x6868; 
  TIM1->CCR1  = 2125; TIM1->CCR2  = 2125; TIM1->CCR3  = 2125;
  TIM1->CCR4  = 4240; 
  TIM1->CCER  = 0xFFF;
  TIM1->BDTR  = 0x80A0; // The ~940ns Dead-Time Fix (Ice cold)
  TIM1->CR2   = 0xF3F;
  TIM1->CR2  &= ~TIM_CR2_MMS_Msk;
  TIM1->CR2  |= (0x7 << TIM_CR2_MMS_Pos); 
  TIM1->EGR  |= 1; TIM1->CR1  |= 1;
}

void setup_bare_metal_ADCs() {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC3EN | RCC_AHB2ENR_ADC12EN;
  GPIOA->MODER |= (3 << 2) | (3 << 4) | (3 << 8) | (3 << 12) | (3 << 14);
  GPIOB->MODER |= (3 << 0);
  GPIOA->PUPDR &= ~((3 << 4) | (3 << 8) | (3 << 12));

  DAC3->CR &= ~DAC_CR_EN1; DAC3->MCR &= ~DAC_MCR_MODE1_Msk; DAC3->MCR |= (3 << DAC_MCR_MODE1_Pos); 
  DAC3->CR |= DAC_CR_EN1; DAC3->DHR12R1 = 2048; 
  OPAMP1->CSR = (3 << 5) | (3 << 2); 
  __asm__("nop"); __asm__("nop"); OPAMP1->CSR |= (1 << 0);

  ADC12_COMMON->CCR &= ~(3 << 16); ADC12_COMMON->CCR |= (2 << 16);
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL); delay(1);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));
  
  ADC1->SMPR1 |= (6 << 6); ADC1->SMPR2 |= (6 << 15); ADC2->SMPR1 |= (6 << 12);
  ADC1->JSQR = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15);
  ADC2->JSQR = (0 << 0) | (1 << 7) | (4 << 9);
  ADC1->CR |= ADC_CR_JADSTART; ADC2->CR |= ADC_CR_JADSTART;
}

void setInverterVoltages(float v_alpha, float v_beta) {
  float Va_raw = v_alpha;
  float Vb_raw = -0.5f * v_alpha + 0.8660254f * v_beta;
  float Vc_raw = -0.5f * v_alpha - 0.8660254f * v_beta;

  float v_min = fminf(Va_raw, fminf(Vb_raw, Vc_raw));
  float v_max = fmaxf(Va_raw, fmaxf(Vb_raw, Vc_raw));
  float v_com = -(v_max + v_min) * 0.5f; 

  TIM1->CCR1 = (uint16_t)(constrain(((Va_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.02f, 0.98f) * 4250.0f);
  TIM1->CCR2 = (uint16_t)(constrain(((Vb_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.02f, 0.98f) * 4250.0f);
  TIM1->CCR3 = (uint16_t)(constrain(((Vc_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.02f, 0.98f) * 4250.0f);
  TIM1->CCER = 0xFFF; 
}

void playTone(float freq, int duration_ms) {
  int half_period = 1000000 / (freq * 2);
  uint32_t start_time = millis();
  while (millis() - start_time < duration_ms) {
    setInverterVoltages(0.6f, 0.0f); 
    delayMicroseconds(half_period);
    setInverterVoltages(-0.6f, 0.0f); 
    delayMicroseconds(half_period);
  }
  setInverterVoltages(0.0f, 0.0f); 
  delay(50);
}

void readELRS() {
  static uint8_t crsf_buf[64];
  static uint8_t crsf_idx = 0;
  static uint32_t last_packet_ms = 0;
  
  if (USART1->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
      USART1->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
  }
  
  while (ELRS_Serial.available()) {
    uint8_t b = ELRS_Serial.read();
    if (crsf_idx == 0 && b != 0xC8) continue; 
    crsf_buf[crsf_idx++] = b;
    
    if (crsf_idx >= 2 && crsf_idx >= crsf_buf[1] + 2) { 
        if (crsf_buf[2] == 0x16) { 
          last_packet_ms = millis();
          
          uint16_t ch[4];
          ch[0] = (crsf_buf[3] | crsf_buf[4] << 8) & 0x07FF;
          ch[1] = (crsf_buf[4] >> 3 | crsf_buf[5] << 5) & 0x07FF;
          ch[2] = (crsf_buf[5] >> 6 | crsf_buf[6] << 2 | crsf_buf[7] << 10) & 0x07FF;
          ch[3] = (crsf_buf[7] >> 1 | crsf_buf[8] << 7) & 0x07FF;

          float raw_thr = (ch[THROTTLE_CH] - 172) / (1811.0f - 172.0f);
          if (raw_thr < 0.02f) raw_thr = 0.0f; 
          if (raw_thr > 1.0f)  raw_thr = 1.0f;
          elrs_throttle = raw_thr;
          
          if (!is_armed && elrs_throttle == 0.0f) {
             is_armed = true;
             playTone(880,  250); delay(50); playTone(1175, 400); 
          }
        }
        crsf_idx = 0; 
    }
    if (crsf_idx >= 64) crsf_idx = 0; 
  }
  if (millis() - last_packet_ms > 200) elrs_throttle = 0.0f;
}

inline void get_Fast_Sin_Cos(float angle_rad, float* out_sin, float* out_cos) {
  while(angle_rad > _PI) angle_rad -= _2PI;
  while(angle_rad < -_PI) angle_rad += _2PI;
  int32_t angle_q31 = (int32_t)((angle_rad / _PI) * 2147483648.0f);
  CORDIC->WDATA = angle_q31;
  *out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
  *out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
}

// Active Flux Observer
void execute_Flux_Observer(float I_U, float I_V) {
  float I_alpha = I_U;
  float I_beta  = (I_U + 2.0f * I_V) * 0.577350269f;

  t_data.i_alpha = I_alpha;
  t_data.i_beta = I_beta;

  float e_alpha = cmd_v_alpha - (RS * I_alpha);
  float e_beta  = cmd_v_beta  - (RS * I_beta);

  flux_alpha += e_alpha * LOOP_DT;
  flux_beta  += e_beta  * LOOP_DT;

  float current_flux_sq = (flux_alpha * flux_alpha) + (flux_beta * flux_beta);
  float target_flux_sq = LAMBDA * LAMBDA;
  float flux_err = target_flux_sq - current_flux_sq;
  float gamma = 15000.0f; 
  
  flux_alpha += gamma * flux_alpha * flux_err * LOOP_DT;
  flux_beta  += gamma * flux_beta  * flux_err * LOOP_DT;

  t_data.flux_alpha = flux_alpha;
  t_data.flux_beta = flux_beta;

  float rotor_flux_alpha = flux_alpha - (LS * I_alpha);
  float rotor_flux_beta  = flux_beta  - (LS * I_beta);

  estimated_angle = atan2f(rotor_flux_beta, rotor_flux_alpha);
  if (estimated_angle < 0.0f) estimated_angle += _2PI;
  t_data.est_angle = estimated_angle;

  static float prev_angle = 0.0f;
  float delta_angle = estimated_angle - prev_angle;
  if (delta_angle < -_PI) delta_angle += _2PI;
  else if (delta_angle > _PI) delta_angle -= _2PI;

  float raw_speed = delta_angle / LOOP_DT;
  estimated_speed = (0.95f * estimated_speed) + (0.05f * raw_speed); 
  prev_angle = estimated_angle;
}

// ==========================================
// MAIN
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(2000000); 
  delay(1000);

  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  CORDIC->CSR = (6 << 4) | (1 << 18); 

  setup_bare_metal_ADCs();
  configureTIM1();
  
  ELRS_Serial.setRx(PB7);
  ELRS_Serial.setTx(PB6);
  ELRS_Serial.begin(420000);

  TIM1->CCR1 = 2125; TIM1->CCR2 = 2125; TIM1->CCR3 = 2125;
  delay(200); 
  long sum_U = 0, sum_V = 0;
  for (int i = 0; i < 4000; i++) {
    while (!(TIM1->SR & TIM_SR_UIF)); TIM1->SR &= ~TIM_SR_UIF;           
    sum_U += ADC1->JDR1; sum_V += ADC1->JDR2;  
  }
  offset_U = (float)sum_U / 4000.0f; offset_V = (float)sum_V / 4000.0f;

  GPIOA->MODER  = (GPIOA->MODER  & ~0x3F0000)   | 0x2A0000;
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF)      | 0x666;
  GPIOB->MODER  = (GPIOB->MODER  & ~0xFC000000) | 0xA8000000;
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  playTone(1175, 150); playTone(1319, 150); playTone(1397, 150); 
}

void loop() {
  static uint32_t tick_counter = 0;
  static float commutation_angle = 0.0f;
  
  // 1. HARDWARE SYNC
  while (!(TIM1->SR & TIM_SR_UIF));
  TIM1->SR &= ~TIM_SR_UIF; 

  uint32_t start_cycles = DWT->CYCCNT; 

  // 2. READ CURRENTS
  float I_U = (float)((int32_t)ADC1->JDR1 - (int)offset_U) * NOMINAL_AMPS_PER_COUNT;
  float I_V = (float)((int32_t)ADC1->JDR2 - (int)offset_V) * NOMINAL_AMPS_PER_COUNT;

  // 3. OBSERVER PRE-LOAD
  execute_Flux_Observer(I_U, I_V);

  // 4. PARSE RC PROTOCOL
  readELRS();

  // 5. DRIVE STATE MACHINE
  switch (current_state) {
    case STATE_OFF:
      setInverterVoltages(0.0f, 0.0f);
      cmd_v_alpha = 0.0f; cmd_v_beta = 0.0f;
      if (is_armed && elrs_throttle > 0.05f) {
         current_state = STATE_ALIGN;
         tick_counter = 0;
      }
      break;

    case STATE_ALIGN:
      {
        float align_v = (float)tick_counter / 10000.0f * 1.5f; 
        if (align_v > 1.5f) align_v = 1.5f;
        cmd_v_alpha = align_v; cmd_v_beta  = 0.0f;
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);
        
        commutation_angle = 0.0f; // Lock angle mechanically

        if (tick_counter > 10000) { 
          tick_counter = 0;
          current_state = STATE_SPOOL;
        }
        if (elrs_throttle == 0.0f) current_state = STATE_OFF;
      }
      break;

    case STATE_SPOOL:
      {
        // 1. Constant Open Loop Slip-Spin (The Old Code Trick)
        commutation_angle += 350.0f * LOOP_DT; // Flat 350 rad/s
        if (commutation_angle > _2PI) commutation_angle -= _2PI;

        float sin_ol, cos_ol;
        get_Fast_Sin_Cos(commutation_angle, &sin_ol, &cos_ol);

        // 2. Strong Vq to drag the rotor
        float v_spool = 2.0f; 
        cmd_v_alpha = -v_spool * sin_ol;
        cmd_v_beta  =  v_spool * cos_ol;
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        // 3. The SNAP (After 1.5 seconds)
        if (tick_counter > 30000) {
          commutation_angle = estimated_angle; // INSTANT TELEPORT
          current_state = STATE_CLOSED_LOOP_FOC;
        }
        if (elrs_throttle == 0.0f) current_state = STATE_OFF;
      }
      break;

    case STATE_CLOSED_LOOP_FOC:
      {
        // 1. The Rubber Band (PLL Tracking)
        float angle_err = estimated_angle - commutation_angle;
        if (angle_err > _PI) angle_err -= _2PI;
        else if (angle_err < -_PI) angle_err += _2PI;

        commutation_angle += angle_err * 0.05f; // Pull Commutation toward Observer
        commutation_angle += estimated_speed * LOOP_DT; // Feedforward
        
        if (commutation_angle > _2PI) commutation_angle -= _2PI;
        else if (commutation_angle < 0.0f) commutation_angle += _2PI;

        float sin_cl, cos_cl;
        get_Fast_Sin_Cos(commutation_angle, &sin_cl, &cos_cl);
        
        // 2. Throttle Mapping
        float MIN_VQ = 1.5f;   
        float MAX_VQ = 11.0f;  
        cmd_vq_throttle = MIN_VQ + (elrs_throttle * (MAX_VQ - MIN_VQ));

        cmd_v_alpha = -cmd_vq_throttle * sin_cl;
        cmd_v_beta  =  cmd_vq_throttle * cos_cl;
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (elrs_throttle == 0.0f) current_state = STATE_OFF;
      }
      break;
  }

  uint32_t end_cycles = DWT->CYCCNT; 
  t_data.comm_angle = commutation_angle; 

  // 6. BINARY TELEMETRY DUMP (Decimated to 1kHz)
  tick_counter++;
  if (tick_counter % 20 == 0) { 
    t_data.loop_cycles = end_cycles - start_cycles;
    t_data.v_alpha = cmd_v_alpha;
    t_data.v_beta = cmd_v_beta;
    t_data.state = current_state;
    Serial.write((uint8_t*)&t_data, sizeof(TelemetryData));
  }
}