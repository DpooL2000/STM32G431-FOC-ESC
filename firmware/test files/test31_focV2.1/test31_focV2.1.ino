// =================================================================
// BARE-METAL SENSORLESS FOC & STARTUP KERNEL V3.1
// Target: WeAct STM32G431 | Fixed ZOH SMO | Debounced Handover Lock
// =================================================================
#include <Arduino.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

// --- CALIBRATED SYSTEM PARAMETERS ---
const float POWER_SUPPLY    = 12.0f;
const float SMO_AD          = 0.23752f;    // Stable ZOH State Feedback Matrix Coeff
const float SMO_BD          = 4.14391f;    // Stable ZOH Input Matrix Coeff
const float LAMBDA          = 0.00241f;    // Identified Flux Linkage (V.s/rad)
const float GAIN_V          = 0.94210f;    // Balanced Phase V Current Scalar
const float LOOP_DT         = 50e-6f;      // 20kHz Execution Period (50 microseconds)

// --- PLL TUNING PARAMETERS ---
const float PLL_KP          = 250.0f;      // Position tracking error gain
const float PLL_KI          = 12500.0f;    // Velocity integration gain

// --- HARDWARE TELEMETRY CONSTANTS ---
float offset_U = 2079.6f;
float offset_V = 2079.4f;
const float NOMINAL_AMPS_PER_COUNT = (3.3f / 4095.0f) / (0.002f * 20.0f); 

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
float open_loop_voltage     = 1.1f;  
float commanded_iq          = 0.0f;  

// --- OBSERVER STATE VECTORS ---
float I_hat_alpha = 0.0f, I_hat_beta = 0.0f;
float E_alpha_fil = 0.0f, E_beta_fil = 0.0f;

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

  TIM1->CR1   = 0x20; TIM1->PSC = 0; TIM1->ARR = 4250; 
  TIM1->CCMR1 = 0x6868; TIM1->CCMR2 = 0x68;
  TIM1->CCER  = 0xFFF; 
  TIM1->BDTR  = 0x80A0; 
  TIM1->CR2   = 0xF3F; TIM1->CR2 &= ~TIM_CR2_MMS_Msk; TIM1->CR2 |= (0x2 << TIM_CR2_MMS_Pos); 
  TIM1->EGR  |= 1; TIM1->CR1 |= 1;
}

void setup_bare_metal_ADCs() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC1EN | RCC_AHB2ENR_ADC12EN;
  GPIOA->MODER |= (3 << 2) | (3 << 8) | (3 << 12) | (3 << 14); GPIOB->MODER |= (3 << 0);
  DAC1->CR |= 1; DAC1->DHR12R1 = 2048;
  ADC12_COMMON->CCR &= ~(3 << 16); ADC12_COMMON->CCR |= (1 << 16);
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD; delay(1);
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN; delay(1);
  ADC1->CR &= ~ADC_CR_ADCALDIF; ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
  ADC2->CR &= ~ADC_CR_ADCALDIF; ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL); delay(1);
  ADC1->ISR |= ADC_ISR_ADRDY; ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
  ADC2->ISR |= ADC_ISR_ADRDY; ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));
  ADC1->SMPR1 |= (3 << 6); ADC1->SMPR2 |= (3 << 15); ADC2->SMPR1 |= (3 << 12); 
  ADC1->JSQR = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15); 
  ADC2->JSQR = (0 << 0) | (1 << 7) | (4 << 9);
  ADC1->CR |= ADC_CR_JADSTART; ADC2->CR |= ADC_CR_JADSTART;
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
// CORE SENSORLESS TRACKING ENGINE
// ==========================================
void execute_Sensorless_SMO_PLL(uint16_t raw_adc_u, uint16_t raw_adc_v, float v_alpha_cmd, float v_beta_cmd, float sliding_gain) {
  
  float I_U = (float)((int32_t)raw_adc_u - (int)offset_U) * NOMINAL_AMPS_PER_COUNT;
  float I_V = (float)((int32_t)raw_adc_v - (int)offset_V) * NOMINAL_AMPS_PER_COUNT * GAIN_V;
  
  // Forward Clarke
  float I_alpha = I_U;
  float I_beta  = (I_U + 2.0f * I_V) * 0.577350269f;

  // Sliding Mode Current Emulator Error
  float current_err_alpha = I_hat_alpha - I_alpha;
  float current_err_beta  = I_hat_beta  - I_beta;

  float sign_alpha = (current_err_alpha > 0.0f) ? 1.0f : ((current_err_alpha < 0.0f) ? -1.0f : 0.0f);
  float sign_beta  = (current_err_beta > 0.0f)  ? 1.0f : ((current_err_beta < 0.0f)  ? -1.0f : 0.0f);

  float z_alpha = sliding_gain * sign_alpha;
  float z_beta  = sliding_gain * sign_beta;

  // FIX: Removed E_alpha_fil from state prediction step to prevent over-compensation
  I_hat_alpha = (SMO_AD * I_hat_alpha) + SMO_BD * (v_alpha_cmd - z_alpha);
  I_hat_beta  = (SMO_AD * I_hat_beta)  + SMO_BD * (v_beta_cmd  - z_beta);

  // Filter switching dynamics to extract clean Back-EMF vectors (~318Hz cutoff)
  float lpf_g = LOOP_DT / (0.0005f + LOOP_DT);
  E_alpha_fil = (1.0f - lpf_g) * E_alpha_fil + lpf_g * z_alpha;
  E_beta_fil  = (1.0f - lpf_g) * E_beta_fil  + lpf_g * z_beta;

  // Cross-Product CORDIC PLL
  float cos_est, sin_est;
  int32_t angle_q31 = (int32_t)((estimated_angle / _PI) * 2147483648.0f);
  CORDIC->WDATA = angle_q31;
  cos_est = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
  sin_est = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;

  float pll_error = -E_alpha_fil * cos_est - E_beta_fil * sin_est;

  float emf_square = (E_alpha_fil * E_alpha_fil) + (E_beta_fil * E_beta_fil);
  if (emf_square > 0.1f) {
    pll_error = pll_error / sqrtf(emf_square);
  }

  static float pll_integrator = 0.0f;
  pll_integrator += PLL_KI * pll_error * LOOP_DT;
  pll_integrator = constrain(pll_integrator, -3000.0f, 3000.0f); 
  
  estimated_speed = pll_integrator + PLL_KP * pll_error;
  estimated_angle += estimated_speed * LOOP_DT;

  if (estimated_angle >= _2PI) estimated_angle -= _2PI;
  else if (estimated_angle < 0.0f) estimated_angle += _2PI;
}

// =================================================================
// DEDICATED LAMBDA MEASUREMENT STATE MACHINE
// =================================================================
void manage_Drive_Transitions() {
  static uint32_t state_timer = 0;
  float V_alpha = 0.0f, V_beta = 0.0f;

  switch (current_state) {
    
    case STATE_OFF:
      setInverterVoltages(0.0f, 0.0f);
      TIM1->CCER = 0xAAA; 
      break;

    case STATE_ALIGN:
      // Soft alignment to prevent massive current spikes
      V_alpha = 0.6f; 
      V_beta  = 0.0f;
      setInverterVoltages(V_alpha, V_beta);
      
      open_loop_clock_angle = 0.0f;
      open_loop_speed_ramp = 10.0f;
      
      if (millis() - state_timer > 1000) { // 1 second settle
        state_timer = millis();
        current_state = STATE_OPEN_LOOP;
        Serial.println("ALIGN COMPLETE. SPOOLING TO MEASUREMENT SPEED...");
      }
      break;

    case STATE_OPEN_LOOP:
      {
        // 1. Smoothly accelerate to 150 rad/s
        if (open_loop_speed_ramp < 150.0f) {
          open_loop_speed_ramp += 40.0f * LOOP_DT; 
        } else {
          open_loop_speed_ramp = 150.0f; // Hold steady at 150 rad/s
        }

        // 2. V/f Voltage Map - Assumes a generic ~0.015 lambda to ensure it has enough push
        open_loop_voltage = 0.4f + (open_loop_speed_ramp * 0.015f);
        open_loop_voltage = constrain(open_loop_voltage, 0.4f, 3.0f);

        // 3. Drive the open loop angle
        open_loop_clock_angle += open_loop_speed_ramp * LOOP_DT;
        if (open_loop_clock_angle >= _2PI) open_loop_clock_angle -= _2PI;

        float sin_ol, cos_ol;
        get_Fast_Sin_Cos(open_loop_clock_angle, &sin_ol, &cos_ol);
        
        V_alpha = -open_loop_voltage * sin_ol;
        V_beta  =  open_loop_voltage * cos_ol;
        setInverterVoltages(V_alpha, V_beta);

        // Run the observer to track the EMF, but don't use it for control
        execute_Sensorless_SMO_PLL(ADC1->JDR1, ADC1->JDR2, V_alpha, V_beta, 4.0f);
      }
      break;
      
    case STATE_HANDOVER:
    case STATE_CLOSED_LOOP:
        // Unused in measurement mode
        break;
  }
}


// ==========================================
// ARDUINO ENVIRONMENT ENTRIES
// ==========================================
void setup() {
  hardware_lockdown();
  Serial.begin(115200);
  while(!Serial);
  delay(1000);

  Serial.println("\n=================================================");
  Serial.println("    FOC KERNEL V3.1 - Stable Sensorless Core     ");
  Serial.println("=================================================");

  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  CORDIC->CSR = (6 << 4) | (1 << 18); 

  setup_bare_metal_ADCs();
  configureTIM1();

  current_state = STATE_ALIGN;
}

void loop() {
  static uint32_t last_tick = 0;
  
  if (micros() - last_tick >= 50) {
    last_tick = micros();
    manage_Drive_Transitions();
  }

  // --- HIGH SPEED LAMBDA MEASUREMENT TELEMETRY ---
  static uint32_t log_timer = 0;
  if (millis() - log_timer > 100) { 
    log_timer = millis();
    
    // Only print once the motor has finished ramping and is holding steady
    if (current_state == STATE_OPEN_LOOP && open_loop_speed_ramp >= 149.0f) {
        
        // Calculate the magnitude of the Back-EMF vector
        float e_mag = sqrtf((E_alpha_fil * E_alpha_fil) + (E_beta_fil * E_beta_fil));
        
        // Calculate Lambda: E_mag / omega
        float calc_lambda = e_mag / 150.0f;
        
        Serial.print("EMF_Mag: "); Serial.print(e_mag, 3);
        Serial.print("V | Speed: 150 rad/s | Calculated Lambda: "); Serial.println(calc_lambda, 6);
    }
  }
}

