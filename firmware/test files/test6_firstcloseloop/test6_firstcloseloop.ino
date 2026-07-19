#include <SimpleFOC.h>

// Variables for dynamic offset tracking
float startup_vref = 0.0;
float startup_offset_a = 0.0;
float startup_offset_b = 0.0;
float startup_offset_c = 0.0;
float filtered_live_vref = 0.0;

// 1. Motor Configuration
BLDCMotor motor = BLDCMotor(7); 

// 2. The Trojan Horse Driver: We only give it PA8, PA9, PA10 so it compiles.
// We will completely overwrite the hardware timer in setup().
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

// 3. Current Sense
InlineCurrentSense current_sense = InlineCurrentSense(0.002f, 50.0f, PA1, PA7, PB0);

float target_velocity = 0.0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

// =================================================================
// YOUR PURE BARE-METAL 6-PWM CONFIGURATION FUNCTION
// =================================================================
void configureTIM1_BareMetal() {
  // A. Lock GPIOs to Alternate Function 6 (TIM1 CH1/2/3) and AF4 (TIM1 CH3N)
  GPIOA->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk);
  GPIOA->MODER |=  (2 << GPIO_MODER_MODE8_Pos) | (2 << GPIO_MODER_MODE9_Pos) | (2 << GPIO_MODER_MODE10_Pos);
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0x00000FFF) | (6<<0) | (6<<4) | (6<<8);

  GPIOB->MODER &= ~(GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk);
  GPIOB->MODER |=  (2 << GPIO_MODER_MODE13_Pos) | (2 << GPIO_MODER_MODE14_Pos) | (2 << GPIO_MODER_MODE15_Pos);
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | (6<<20) | (6<<24) | (4<<28);

  // B. Enable Timer 1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  // C. Base Timer Setup (Center-aligned, 170MHz / 4250 = 20kHz)
  TIM1->CR1   = TIM_CR1_CMS_0;          
  TIM1->PSC   = 0;                      
  TIM1->ARR   = 4250;                   

  // D. Configure Output Compare Modes
  TIM1->CCMR1 = (6<<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                (6<<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = (6<<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

  // E. POLARITY HACK: Active-Low for your 6N137 Inverters!
  TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC1P | TIM_CCER_CC1NP |
                TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC2P | TIM_CCER_CC2NP |
                TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3P | TIM_CCER_CC3NP;

  // F. SAFETY IDLE SHIELD: Force 3.3V (HIGH) when PWM is disabled to keep MOSFETs OFF
  TIM1->CR2   = TIM_CR2_OIS1 | TIM_CR2_OIS1N |
                TIM_CR2_OIS2 | TIM_CR2_OIS2N |
                TIM_CR2_OIS3 | TIM_CR2_OIS3N;

  // G. HARDWARE DEAD-TIME: ~588ns
  TIM1->BDTR  = TIM_BDTR_MOE | (100 << TIM_BDTR_DTG_Pos);

  // H. Start Timer
  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
  TIM1->EGR  = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
}

void setup() {
  // --- 1.65V HARDWARE HACK ---
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC1EN;
  GPIOA->MODER |= (3 << GPIO_MODER_MODE4_Pos) | (3 << GPIO_MODER_MODE6_Pos);
  DAC1->CR &= ~DAC_CR_EN1;             
  DAC1->MCR &= ~DAC_MCR_MODE1_Msk;     
  DAC1->CR |= DAC_CR_EN1;              
  DAC1->DHR12R1 = 2048;                
  
  Serial.begin(115200);
  analogReadResolution(12);

  // --- DRIVER CONFIGURATION ---
  driver.voltage_power_supply = 12.0;
  driver.voltage_limit = 2.0; 
  driver.init();

  // =======================================================
  // HIJACK THE HARDWARE!
  // Overwrite everything SimpleFOC just did to Timer 1
  // =======================================================
  configureTIM1_BareMetal();
  
  motor.linkDriver(&driver);

  // --- CURRENT SENSE CONFIGURATION ---
  if(!current_sense.init()){
    Serial.println("Current sense init failed.");
    return;
  }
  motor.linkCurrentSense(&current_sense);

  // === CAPTURE THE BASELINE OFFSETS ===
  startup_offset_a = current_sense.offset_ia;
  startup_offset_b = current_sense.offset_ib;
  startup_offset_c = current_sense.offset_ic;

  float vref_sum = 0;
  for(int i=0; i<50; i++) {
    vref_sum += (analogRead(PA6) / 4095.0) * 3.3;
    delay(1);
  }
  startup_vref = vref_sum / 50.0;
  filtered_live_vref = startup_vref; 

  // --- MOTOR CONFIGURATION ---
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 2.0; 
  motor.velocity_limit = 20.0; 

  motor.init();
  command.add('T', doTarget, "target velocity");
  Serial.println("Motor ready. Type 'T2' to spin at 2 rad/s.");
}

void loop() {
  // === DYNAMIC OFFSET TRACKING ===
  float raw_vref = (analogRead(PA6) / 4095.0) * 3.3;
  filtered_live_vref = (0.01 * raw_vref) + (0.99 * filtered_live_vref);
  float delta_vref = filtered_live_vref - startup_vref;

  current_sense.offset_ia = startup_offset_a + delta_vref;
  current_sense.offset_ib = startup_offset_b + delta_vref;
  current_sense.offset_ic = startup_offset_c + delta_vref;

  motor.loopFOC();
  motor.move(target_velocity);

  PhaseCurrent_s currents = current_sense.getPhaseCurrents();

  static unsigned long last_plot_time = 0;
  if (millis() - last_plot_time > 20) { 
    Serial.print("Amps_A:"); Serial.print(currents.a); Serial.print(",");
    Serial.print("Amps_B:"); Serial.print(currents.b); Serial.print(",");
    Serial.print("Amps_C:"); Serial.println(currents.c);
    last_plot_time = millis();
  }

  command.run();
}