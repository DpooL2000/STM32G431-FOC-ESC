#include <SimpleFOC.h>

// --- HARDWARE PINS ---
const int BEMF_A = PA0;
const int BEMF_B = PA3;
const int BEMF_C = PB1;

// --- SYSTEM CONSTANTS ---
const float SQRT3_2 = 0.866025f;
const float VOLTAGE_LIMIT = 2.0f;  // Max open-loop driving voltage
const float POWER_SUPPLY = 12.0f;
const float BEMF_MULTIPLIER = (3.3f / 4095.0f) * 11.0f; // Converts 12-bit ADC to real Volts

// --- VARIABLES ---
float target_velocity = 0.0; 
float electrical_angle = 0.0;
unsigned long last_time = 0;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

// =================================================================
// YOUR PERFECT BARE-METAL TIM1 CONFIGURATION
// =================================================================
void configureTIM1_BareMetal() {
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

  // ACTIVE LOW POLARITY FOR 6N137
  TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC1P | TIM_CCER_CC1NP |
                TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC2P | TIM_CCER_CC2NP |
                TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3P | TIM_CCER_CC3NP;

  // SAFETY IDLE SHIELD
  TIM1->CR2   = TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_OIS2 | TIM_CR2_OIS2N | TIM_CR2_OIS3 | TIM_CR2_OIS3N;
  TIM1->BDTR  = TIM_BDTR_MOE | (100 << TIM_BDTR_DTG_Pos);

  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0; 
  TIM1->EGR  = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // --- 1.65V HARDWARE HACK (Keep this running to stabilize the board) ---
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC1EN;
  GPIOA->MODER |= (3 << GPIO_MODER_MODE4_Pos) | (3 << GPIO_MODER_MODE6_Pos);
  DAC1->CR &= ~DAC_CR_EN1;             
  DAC1->MCR &= ~DAC_MCR_MODE1_Msk;     
  DAC1->CR |= DAC_CR_EN1;              
  DAC1->DHR12R1 = 2048;                
  
  // Set BEMF pins to Analog Input Mode
  pinMode(BEMF_A, INPUT_ANALOG);
  pinMode(BEMF_B, INPUT_ANALOG);
  pinMode(BEMF_C, INPUT_ANALOG);

  configureTIM1_BareMetal();

  command.add('T', doTarget, "target velocity");
  last_time = micros();
  
  Serial.println("BEMF Observer Ready. Type 'T10' to spin open-loop at 10 rad/s.");
}

void loop() {
  // =================================================================
  // 1. OPEN-LOOP MOTOR DRIVING (To generate the BEMF)
  // =================================================================
  unsigned long now = micros();
  float dt = (now - last_time) * 1e-6f;
  last_time = now;

  electrical_angle += target_velocity * dt;
  if (electrical_angle < 0.0f) electrical_angle += _2PI;
  if (electrical_angle > _2PI) electrical_angle -= _2PI;

  float Vq = (target_velocity != 0.0f) ? VOLTAGE_LIMIT : 0.0f;
  float Vd = 0.0f; 

  // Math Engine: Park & Clarke
  float c = _cos(electrical_angle);
  float s = _sin(electrical_angle);
  float V_alpha = Vd * c - Vq * s;
  float V_beta  = Vd * s + Vq * c;

  float Va = V_alpha;
  float Vb = -0.5f * V_alpha + SQRT3_2 * V_beta;
  float Vc = -0.5f * V_alpha - SQRT3_2 * V_beta;

  // Push to Silicon
  TIM1->CCR1 = (uint16_t)(_constrain((Va / POWER_SUPPLY) + 0.5f, 0.0f, 1.0f) * 4250.0f);
  TIM1->CCR2 = (uint16_t)(_constrain((Vb / POWER_SUPPLY) + 0.5f, 0.0f, 1.0f) * 4250.0f);
  TIM1->CCR3 = (uint16_t)(_constrain((Vc / POWER_SUPPLY) + 0.5f, 0.0f, 1.0f) * 4250.0f);


  // =================================================================
  // 2. THE BEMF OBSERVER (Dynamic Neutral Calculation)
  // =================================================================
  
  // Read Raw Analog Values
  float raw_A = analogRead(BEMF_A);
  float raw_B = analogRead(BEMF_B);
  float raw_C = analogRead(BEMF_C);

  // Convert to Real Volts based on your resistor dividers
  float volts_A = raw_A * BEMF_MULTIPLIER;
  float volts_B = raw_B * BEMF_MULTIPLIER;
  float volts_C = raw_C * BEMF_MULTIPLIER;

  // Calculate the Virtual Star Point (Dynamic Neutral)
  float neutral = (volts_A + volts_B + volts_C) / 3.0f;

  // Calculate the pure BEMF relative to neutral
  float zero_cross_A = volts_A - neutral;
  float zero_cross_B = volts_B - neutral;
  float zero_cross_C = volts_C - neutral;

  // =================================================================
  // 3. TELEMETRY
  // =================================================================
  static unsigned long last_plot_time = 0;
  if (millis() - last_plot_time > 10) { 
    // Plotting the Zero-Crossing values. They should oscillate perfectly around 0.00
    Serial.print("BEMF_A:"); Serial.print(zero_cross_A); Serial.print(",");
    Serial.print("BEMF_B:"); Serial.print(zero_cross_B); Serial.print(",");
    Serial.print("BEMF_C:"); Serial.print(zero_cross_C); Serial.print(",");
    Serial.print("ZeroLine:"); Serial.println(0.0f); // Prints a flat line at 0V for visual reference
    last_plot_time = millis();
  }

  command.run();
}