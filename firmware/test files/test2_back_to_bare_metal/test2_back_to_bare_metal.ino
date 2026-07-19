#include <math.h>

// BEMF Sensing Pins for the Plotter
const int BEMF_A = PA0;
const int BEMF_B = PA3;
const int BEMF_C = PB1;

float filter_alpha = 0.05; 
float filtered_A = 0.0, filtered_B = 0.0, filtered_C = 0.0;

// SPWM Variables
float electrical_angle = 0.0;
float speed = 0.0; // Controlled by Serial Monitor
float amplitude = 0.15; // Limits max voltage to ~15% of 12V (1.8V safe limit)

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // =======================================================
  // 1. INITIAL SAFETY LOCK (Standard GPIO Mode)
  // =======================================================
  // Enable GPIO Clocks
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

  // Set PA8, PA9, PA10 to OUTPUT
  GPIOA->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk);
  GPIOA->MODER |= (1 << GPIO_MODER_MODE8_Pos) | (1 << GPIO_MODER_MODE9_Pos) | (1 << GPIO_MODER_MODE10_Pos);
  
  // Set PB13, PB14, PB15 to OUTPUT
  GPIOB->MODER &= ~(GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk);
  GPIOB->MODER |= (1 << GPIO_MODER_MODE13_Pos) | (1 << GPIO_MODER_MODE14_Pos) | (1 << GPIO_MODER_MODE15_Pos);

  // Drive all HIGH (6N137 LEDs ON -> IR2101 Gates OFF)
  GPIOA->ODR |= (1 << 8) | (1 << 9) | (1 << 10);
  GPIOB->ODR |= (1 << 13) | (1 << 14) | (1 << 15);
  delay(100); // Let voltages stabilize

  // =======================================================
  // 2. TRANSFER TO HARDWARE TIMER (Alternate Function 6/4)
  // =======================================================
  // Switch to Alternate Function Mode (10)
  GPIOA->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk);
  GPIOA->MODER |= (2 << GPIO_MODER_MODE8_Pos) | (2 << GPIO_MODER_MODE9_Pos) | (2 << GPIO_MODER_MODE10_Pos);
  
  GPIOB->MODER &= ~(GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk);
  GPIOB->MODER |= (2 << GPIO_MODER_MODE13_Pos) | (2 << GPIO_MODER_MODE14_Pos) | (2 << GPIO_MODER_MODE15_Pos);

  // Map to TIM1
  // PA8, PA9, PA10 = AF6
  GPIOA->AFR[1] &= ~(0x00000FFF); 
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 
  
  // PB13 = AF6, PB14 = AF6, PB15 = AF4 (STM32G4 specific pinout)
  GPIOB->AFR[1] &= ~(0xFFF00000); 
  GPIOB->AFR[1] |= (6 << 20) | (6 << 24) | (4 << 28);

  // =======================================================
  // 3. BARE METAL TIM1 CONFIGURATION
  // =======================================================
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Enable TIM1 Clock

  TIM1->CR1 = TIM_CR1_CMS_0; // Center-aligned mode 1
  TIM1->PSC = 0;             // Run at 170 MHz
  TIM1->ARR = 4250;          // 20kHz PWM Frequency

  // Set to PWM Mode 1
  TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

  // CRITICAL: ACTIVE LOW POLARITY (Matches your 6N137 inversion)
  // When Duty is 0%, pins output HIGH (Safe State).
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC1P | TIM_CCER_CC1NP |
               TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC2P | TIM_CCER_CC2NP |
               TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3P | TIM_CCER_CC3NP;

  // DEAD-TIME GENERATOR (Hardware Enforced)
  // DTG = 100 on a 170MHz clock gives exactly ~588ns of deadtime.
  TIM1->BDTR = TIM_BDTR_MOE | (100 << TIM_BDTR_DTG_Pos); 

  // Initialize to 0% duty (Outputs locked HIGH)
  TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;

  // Start the Timer
  TIM1->CR1 |= TIM_CR1_CEN;
  TIM1->EGR |= TIM_EGR_UG; 
  
  Serial.println("Bare Metal TIM1 Active. Gates are strictly hardware controlled.");
}

void loop() {
  // 1. SERIAL INPUT (To control speed)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    if (input.length() > 0) {
      speed = input.toFloat() * 0.01; 
    }
  }

  // 2. SINUSOIDAL PWM CALCULATION (SPWM)
  electrical_angle += speed;
  if (electrical_angle > 2 * PI) electrical_angle -= 2 * PI;
  if (electrical_angle < 0) electrical_angle += 2 * PI;

  // Calculate 3-phase sine waves
  float duty_A = (sin(electrical_angle) * amplitude) + 0.5;
  float duty_B = (sin(electrical_angle - 2.0944) * amplitude) + 0.5;
  float duty_C = (sin(electrical_angle + 2.0944) * amplitude) + 0.5;

  // Push duties directly into the silicon registers
  TIM1->CCR1 = (uint16_t)(duty_A * 4250);
  TIM1->CCR2 = (uint16_t)(duty_B * 4250);
  TIM1->CCR3 = (uint16_t)(duty_C * 4250);

  // 3. PLOT THE INTERNAL MATH (Every 20ms)
  static unsigned long last_plot_time = 0;
  if (millis() - last_plot_time > 20) { 
    // We multiply by 12 just so it scales nicely on your plotter 0-12V axis
    Serial.print("Phase_A:"); Serial.print(duty_A * 12.0); Serial.print(",");
    Serial.print("Phase_B:"); Serial.print(duty_B * 12.0); Serial.print(",");
    Serial.print("Phase_C:"); Serial.println(duty_C * 12.0);
    last_plot_time = millis();
  }
  
  delay(1); 
}