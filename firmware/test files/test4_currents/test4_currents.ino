#include <math.h>

// Add these pin definitions at the top of your file
const int CURR_A = PA1;
const int CURR_B = PA7;
const int CURR_C = PB0;

const int VREF_MONITOR = PA6; // The secret monitor pin!

// BEMF Sensing Pins for the Plotter
const int BEMF_A = PA0;
const int BEMF_B = PA3;
const int BEMF_C = PB1;

// Add these global variables at the top
bool motor_running = false;
float offset_A = 1.658, offset_B = 1.658, offset_C = 1.658;

float filter_alpha = 0.05; 
float filtered_A = 0.0, filtered_B = 0.0, filtered_C = 0.0;

// SPWM Variables
float electrical_angle = 0.0;
float speed = 0.0;      // Controlled by Serial Monitor
float amplitude = 0.15; // Limits max voltage to ~15% of 12V

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // =======================================================
  // 1. ENABLE CLOCKS (GPIOA, GPIOB, DAC1, TIM1)
  // =======================================================
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC1EN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; 

  // =======================================================
  // 2. THE 1.65V HARDWARE HACK (PA4 outputs, PA6 receives)
  // =======================================================
  // Set both to Analog Mode safely
  GPIOA->MODER |= (3 << GPIO_MODER_MODE4_Pos) | (3 << GPIO_MODER_MODE6_Pos);

  DAC1->CR &= ~DAC_CR_EN1;             
  DAC1->MCR &= ~DAC_MCR_MODE1_Msk;     // Buffer Enabled Mode
  DAC1->CR |= DAC_CR_EN1;              
  DAC1->DHR12R1 = 2048;                // Output 1.65V to PA4

  // =======================================================
  // 3. TRANSFER PINS TO HARDWARE TIMER 1 (Alternate Function)
  // =======================================================
  // PA8, PA9, PA10 -> AF6
  GPIOA->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk);
  GPIOA->MODER |= (2 << GPIO_MODER_MODE8_Pos) | (2 << GPIO_MODER_MODE9_Pos) | (2 << GPIO_MODER_MODE10_Pos);
  GPIOA->AFR[1] &= ~(0x00000FFF); 
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 
  
  // PB13, PB14 -> AF6 | PB15 -> AF4
  GPIOB->MODER &= ~(GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk);
  GPIOB->MODER |= (2 << GPIO_MODER_MODE13_Pos) | (2 << GPIO_MODER_MODE14_Pos) | (2 << GPIO_MODER_MODE15_Pos);
  GPIOB->AFR[1] &= ~(0xFFF00000); 
  GPIOB->AFR[1] |= (6 << 20) | (6 << 24) | (4 << 28);

  // =======================================================
  // 4. BARE METAL TIM1 CONFIGURATION (The Brains)
  // =======================================================
  TIM1->CR1 = TIM_CR1_CMS_0; // Center-aligned mode 1
  TIM1->PSC = 0;             // Run at 170 MHz
  TIM1->ARR = 4250;          // 20kHz PWM Frequency

  // Set to PWM Mode 1
  TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

  // CRITICAL: ACTIVE LOW POLARITY (Matches your 6N137 inversion)
  // When Duty is 0%, pins output HIGH -> 6N137 LED ON -> MOSFET OFF.
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC1P | TIM_CCER_CC1NP |
               TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC2P | TIM_CCER_CC2NP |
               TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3P | TIM_CCER_CC3NP;

  // DEAD-TIME GENERATOR (Hardware Enforced ~588ns)
  TIM1->BDTR = TIM_BDTR_MOE | (100 << TIM_BDTR_DTG_Pos); 

  // Initialize to 0% duty (Outputs locked HIGH / Safe)
  TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;

  // Start the Timer
  TIM1->CR1 |= TIM_CR1_CEN;
  TIM1->EGR |= TIM_EGR_UG; 
  
  Serial.println("System Ready. Bare-Metal SPWM Active.");
}


void loop() {
  // 1. SERIAL INPUT
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    if (input.length() > 0) {
      speed = input.toFloat() * 0.01; 
      if (speed != 0) motor_running = true;
      else motor_running = false;
      Serial.print("Speed set to: "); Serial.println(speed);
    }
  }

  // 2. SAFE IDLE LOGIC
  if (motor_running) {
    amplitude = 0.05; // 5% voltage while running
    electrical_angle += speed;
    if (electrical_angle > 2 * PI) electrical_angle -= 2 * PI;
    if (electrical_angle < 0) electrical_angle += 2 * PI;
  } else {
    amplitude = 0.0; // 0% voltage difference while stopped (0 Amps!)
  }

  float duty_A = (sin(electrical_angle) * amplitude) + 0.5;
  float duty_B = (sin(electrical_angle - 2.0944) * amplitude) + 0.5;
  float duty_C = (sin(electrical_angle + 2.0944) * amplitude) + 0.5;

  TIM1->CCR1 = (uint16_t)(duty_A * 4250);
  TIM1->CCR2 = (uint16_t)(duty_B * 4250);
  TIM1->CCR3 = (uint16_t)(duty_C * 4250);

  // 3. READ THE DYNAMIC REFERENCE AND CURRENTS
  float live_vref = (analogRead(VREF_MONITOR) / 4095.0) * 3.3;

  float volts_I_A = (analogRead(CURR_A) / 4095.0) * 3.3;
  float volts_I_B = (analogRead(CURR_B) / 4095.0) * 3.3;
  float volts_I_C = (analogRead(CURR_C) / 4095.0) * 3.3;

  // Subtract the LIVE reference voltage, not a hardcoded number!
  float amps_A = (volts_I_A - live_vref) / 0.1;
  float amps_B = (volts_I_B - live_vref) / 0.1;
  float amps_C = (volts_I_C - live_vref) / 0.1;

  filtered_A = (filter_alpha * amps_A) + ((1.0 - filter_alpha) * filtered_A);
  filtered_B = (filter_alpha * amps_B) + ((1.0 - filter_alpha) * filtered_B);
  filtered_C = (filter_alpha * amps_C) + ((1.0 - filter_alpha) * filtered_C);

  static unsigned long last_plot_time = 0;
  if (millis() - last_plot_time > 20) { 
    Serial.print("Live_VREF:"); Serial.print(live_vref); Serial.print(",");
    Serial.print("Amps_A:"); Serial.print(filtered_A); Serial.print(",");
    Serial.print("Amps_B:"); Serial.print(filtered_B); Serial.print(",");
    Serial.print("Amps_C:"); Serial.println(filtered_C);
    last_plot_time = millis();
  }
  
  delay(1); 
}