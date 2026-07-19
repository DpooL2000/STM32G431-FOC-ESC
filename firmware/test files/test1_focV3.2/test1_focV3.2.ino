// =================================================================
// BARE-METAL SENSORLESS FOC KERNEL V3.2
// Target: WeAct STM32G431 | Active Flux Observer | JSQR Precision
// =================================================================
#include <Arduino.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

// --- CALIBRATED SYSTEM PARAMETERS ---
const float POWER_SUPPLY    = 12.0f;
const float LAMBDA          = 0.00241f;    // Identified Flux Linkage (V.s/rad)
const float GAIN_V          = 1.0f;        // Balanced Phase V Current Scalar
const float LOOP_DT         = 50e-6f;      // 20kHz Execution Period (50 microseconds)

// --- MOTOR PHYSICAL CONSTANTS (MUST BE MEASURED FOR YOUR MOTOR) ---
const float RS = 0.05f;      // Phase Resistance in Ohms (Example: 50mOhm)
const float LS = 0.00001f;   // Phase Inductance in Henrys (Example: 10uH)

// --- HARDWARE TELEMETRY CONSTANTS ---
const float SHUNT_RESISTOR  = 0.002f;
const float AMP_GAIN        = 20.0f;
const float VOLTS_PER_COUNT = 3.3f / 4095.0f;
const float NOMINAL_AMPS_PER_COUNT = VOLTS_PER_COUNT / (SHUNT_RESISTOR * AMP_GAIN);

// Dynamic Boot Calibration Offsets
float offset_U = 2103.6f;
float offset_V = 2103.6f;
float offset_W = 2103.6f;

// --- STATE MACHINE STRUCTS ---
enum DriveState {
  STATE_OFF,
  STATE_ALIGN,
  STATE_OPEN_LOOP,
  STATE_HANDOVER,
  STATE_CLOSED_LOOP
};
DriveState current_state = STATE_OFF;

// --- DYNAMIC CONTROL REGISTERS ---
float open_loop_clock_angle = 0.0f;
float open_loop_speed_ramp  = 10.0f; 
float cmd_v_alpha = 0.0f;
float cmd_v_beta  = 0.0f;

// --- OBSERVER STATE VECTORS ---
float flux_alpha = 0.0f;
float flux_beta  = 0.0f;

// --- SYSTEM SENSORLESS OUTPUTS ---
float estimated_angle = 0.0f;        
float estimated_speed = 0.0f;        

// ==========================================
// HARDWARE REGISTERS INITIALIZATION
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

  TIM1->CR1   = 0x20;   // Center-aligned mode 1
  TIM1->PSC   = 0;
  TIM1->ARR   = 4250;

  TIM1->CCMR1 = 0x6868;
  TIM1->CCMR2 = 0x6868; 

  TIM1->CCR1  = 2125;
  TIM1->CCR2  = 2125;
  TIM1->CCR3  = 2125;
  TIM1->CCR4  = 4240;   // ADC trigger near peak of triangle (Valley of current)

  TIM1->CCER  = 0xFFF;
  TIM1->BDTR  = 0x80A0;

  TIM1->CR2   = 0xF3F;
  TIM1->CR2  &= ~TIM_CR2_MMS_Msk;
  TIM1->CR2  |= (0x7 << TIM_CR2_MMS_Pos); // TRGO = OC4REF

  TIM1->EGR  |= 1;
  TIM1->CR1  |= 1;
}

void setup_bare_metal_ADCs() {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC3EN | RCC_AHB2ENR_ADC12EN;

  GPIOA->MODER |= (3 << 2) | (3 << 4) | (3 << 8) | (3 << 12) | (3 << 14);
  GPIOB->MODER |= (3 << 0);
  GPIOA->PUPDR &= ~((3 << 4) | (3 << 8) | (3 << 12));

  // Precision DAC3 to OPAMP1 Routing
  DAC3->CR &= ~DAC_CR_EN1;
  DAC3->MCR &= ~DAC_MCR_MODE1_Msk;
  DAC3->MCR |= (3 << DAC_MCR_MODE1_Pos); 
  DAC3->CR |= DAC_CR_EN1;
  DAC3->DHR12R1 = 2048; 

  OPAMP1->CSR = (3 << 5) | (3 << 2); 
  __asm__("nop"); __asm__("nop");
  OPAMP1->CSR |= (1 << 0);

  ADC12_COMMON->CCR &= ~(3 << 16); ADC12_COMMON->CCR |= (2 << 16);
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL);
  delay(1);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));
  
  ADC1->SMPR1 |= (6 << 6); ADC1->SMPR2 |= (6 << 15); ADC2->SMPR1 |= (6 << 12);
  
  // Triggered via TIM1_TRGO
  ADC1->JSQR = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15);
  ADC2->JSQR = (0 << 0) | (1 << 7) | (4 << 9);
  
  ADC1->CR |= ADC_CR_JADSTART; ADC2->CR |= ADC_CR_JADSTART;
}

void calibrate_AD8418_offsets() {
  Serial.println("Calibrating offsets... Keep motor still.");
  TIM1->CCR1 = 2125; TIM1->CCR2 = 2125; TIM1->CCR3 = 2125;
  delay(200); 

  long sum_U = 0, sum_V = 0, sum_W = 0;
  const int N = 2000;

  for (int i = 0; i < N; i++) {
    while (!(TIM1->SR & TIM_SR_UIF)); 
    TIM1->SR &= ~TIM_SR_UIF;           
    sum_U += ADC1->JDR1;  
    sum_V += ADC1->JDR2;  
    sum_W += ADC2->JDR1;
  }

  offset_U = (float)sum_U / (float)N;
  offset_V = (float)sum_V / (float)N;
  offset_W = (float)sum_W / (float)N;
  Serial.println("Calibration done.");
}

inline void get_Fast_Sin_Cos(float angle_rad, float* out_sin, float* out_cos) {
  while(angle_rad > _PI) angle_rad -= _2PI;
  while(angle_rad < -_PI) angle_rad += _2PI;
  int32_t angle_q31 = (int32_t)((angle_rad / _PI) * 2147483648.0f);
  CORDIC->WDATA = angle_q31;
  *out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
  *out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
}

void setInverterVoltages(float v_alpha, float v_beta) {
  float Va_raw = v_alpha;
  float Vb_raw = -0.5f * v_alpha + 0.8660254f * v_beta;
  float Vc_raw = -0.5f * v_alpha - 0.8660254f * v_beta;

  float v_min = fminf(Va_raw, fminf(Vb_raw, Vc_raw));
  float v_max = fmaxf(Va_raw, fmaxf(Vb_raw, Vc_raw));
  float v_com = -(v_max + v_min) * 0.5f; 

  TIM1->CCR1 = (uint16_t)(constrain(((Va_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR2 = (uint16_t)(constrain(((Vb_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCR3 = (uint16_t)(constrain(((Vc_raw + v_com) / POWER_SUPPLY) + 0.5f, 0.05f, 0.95f) * 4250.0f);
  TIM1->CCER = 0xFFF; 
}

// ==========================================
// CORE FLUX OBSERVER ENGINE (NO PLL, NO LAG)
// ==========================================
void execute_Flux_Observer(uint16_t raw_adc_u, uint16_t raw_adc_v) {
  
  float I_U = (float)((int32_t)raw_adc_u - (int)offset_U) * NOMINAL_AMPS_PER_COUNT;
  float I_V = (float)((int32_t)raw_adc_v - (int)offset_V) * NOMINAL_AMPS_PER_COUNT * GAIN_V;
  
  float I_alpha = I_U;
  float I_beta  = (I_U + 2.0f * I_V) * 0.577350269f;

  float e_alpha = cmd_v_alpha - (RS * I_alpha);
  float e_beta  = cmd_v_beta  - (RS * I_beta);

  // Core Integration
  flux_alpha += e_alpha * LOOP_DT;
  flux_beta  += e_beta  * LOOP_DT;

  // DC Drift Centering
  float current_flux_sq = (flux_alpha * flux_alpha) + (flux_beta * flux_beta);
  float target_flux_sq = LAMBDA * LAMBDA;
  float flux_err = target_flux_sq - current_flux_sq;
  float gamma = 15000.0f; // Centering Gain
  
  flux_alpha += gamma * flux_alpha * flux_err * LOOP_DT;
  flux_beta  += gamma * flux_beta  * flux_err * LOOP_DT;

  // Remove Inductive Stator Field
  float rotor_flux_alpha = flux_alpha - (LS * I_alpha);
  float rotor_flux_beta  = flux_beta  - (LS * I_beta);

  // Direct Hardware FPU Angle Extraction
  estimated_angle = atan2f(rotor_flux_beta, rotor_flux_alpha);
  if (estimated_angle < 0.0f) estimated_angle += _2PI;

  // Calculate speed for telemetry/debugging only (not used for angle control)
  static float prev_angle = 0.0f;
  float delta_angle = estimated_angle - prev_angle;
  if (delta_angle < -_PI) delta_angle += _2PI;
  else if (delta_angle > _PI) delta_angle -= _2PI;

  float raw_speed = delta_angle / LOOP_DT;
  estimated_speed = (0.95f * estimated_speed) + (0.05f * raw_speed); 
  prev_angle = estimated_angle;
}

// ==========================================
// STATE MACHINE
// ==========================================
void manage_Drive_Transitions() {
  static uint32_t state_timer = 0;

  switch (current_state) {
    
    case STATE_OFF:
      cmd_v_alpha = 0.0f; cmd_v_beta = 0.0f;
      setInverterVoltages(0.0f, 0.0f);
      TIM1->CCER = 0xAAA; 
      break;

    case STATE_ALIGN:
      cmd_v_alpha = 1.0f; 
      cmd_v_beta  = 0.0f;
      setInverterVoltages(cmd_v_alpha, cmd_v_beta);
      open_loop_clock_angle = 0.0f;
      open_loop_speed_ramp = 10.0f;
      
      if (millis() - state_timer > 1000) { 
        state_timer = millis();
        current_state = STATE_OPEN_LOOP;
        Serial.println("ALIGN COMPLETE. SPOOLING...");
      }
      break;

    case STATE_OPEN_LOOP:
      {
        if (open_loop_speed_ramp < 150.0f) {
          open_loop_speed_ramp += 40.0f * LOOP_DT; 
        } 
        
        float open_loop_voltage = 0.4f + (open_loop_speed_ramp * 0.015f);
        open_loop_voltage = constrain(open_loop_voltage, 0.4f, 3.0f);

        open_loop_clock_angle += open_loop_speed_ramp * LOOP_DT;
        if (open_loop_clock_angle >= _2PI) open_loop_clock_angle -= _2PI;

        float sin_ol, cos_ol;
        get_Fast_Sin_Cos(open_loop_clock_angle, &sin_ol, &cos_ol);
        
        cmd_v_alpha = -open_loop_voltage * sin_ol;
        cmd_v_beta  =  open_loop_voltage * cos_ol;
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        // Handover Trigger: Once we hit 150 rad/s and hold for 1 second
        if (open_loop_speed_ramp >= 150.0f && (millis() - state_timer > 2000)) {
           current_state = STATE_CLOSED_LOOP;
           Serial.println("HANDOVER TO FLUX OBSERVER ACTIVE!");
        }
      }
      break;
      
    case STATE_CLOSED_LOOP:
      {
        // Drive strictly off the Flux Observer's absolute angle
        float sin_cl, cos_cl;
        get_Fast_Sin_Cos(estimated_angle, &sin_cl, &cos_cl);
        
        float vq_cmd = 2.5f; // Closed-loop target voltage (Torque proxy)
        float vd_cmd = 0.0f; // Keep D-axis voltage at 0 for max efficiency
        
        cmd_v_alpha = vd_cmd * cos_cl - vq_cmd * sin_cl;
        cmd_v_beta  = vd_cmd * sin_cl + vq_cmd * cos_cl;
        
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);
      }
      break;
  }
}

// ==========================================
// MAIN
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=================================================");
  Serial.println("    FOC KERNEL V3.2 - ACTIVE FLUX OBSERVER       ");
  Serial.println("=================================================");

  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  CORDIC->CSR = (6 << 4) | (1 << 18); 

  setup_bare_metal_ADCs();
  configureTIM1();
  calibrate_AD8418_offsets();

  // Restore AF pin modes for hardware logic
  GPIOA->MODER  = (GPIOA->MODER  & ~0x3F0000)   | 0x2A0000;
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF)      | 0x666;
  GPIOB->MODER  = (GPIOB->MODER  & ~0xFC000000) | 0xA8000000;
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000;

  current_state = STATE_ALIGN;
}

void loop() {
  // 1. HARDWARE SYNC: Wait for the exact valley of the PWM
  // This physically guarantees our FOC loop runs at exactly 20kHz
  while (!(TIM1->SR & TIM_SR_UIF));
  TIM1->SR &= ~TIM_SR_UIF; 

  // 2. STATE MACHINE: Calculate target voltages and write to PWM
  manage_Drive_Transitions();

  // 3. FLUX OBSERVER: Calculate the angle for the *next* cycle
  // Reads JDR registers instantly. No DMA latency.
  execute_Flux_Observer(ADC1->JDR1, ADC1->JDR2);

  // 4. TELEMETRY LOGGING
  static uint32_t log_timer = 0;
  if (millis() - log_timer > 100) { 
    log_timer = millis();
    if (current_state == STATE_CLOSED_LOOP) {
        Serial.print("Target Vq: 2.5V | Speed: "); 
        Serial.print(estimated_speed);
        Serial.print(" rad/s | Angle: ");
        Serial.println(estimated_angle);
    }
  }
}