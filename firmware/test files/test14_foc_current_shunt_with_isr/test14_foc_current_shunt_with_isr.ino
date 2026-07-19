// --- GLOBAL VARIABLES ---
int step = 1;
int pwmValue = 850; // 20% Duty Cycle (Lowered to reduce vibration)
int stepDelay = 4;  // Milliseconds between steps (Speed control)

// Volatile variables because they are updated in the background by the hardware interrupt
volatile float ampsU = 0.0;
volatile float ampsV = 0.0;
volatile float ampsW = 0.0;
volatile bool newDataReady = false;

void setup() {
  Serial.begin(115200);
  
  // Set ADC to 12-bit resolution (0-4095) instead of Arduino's default 10-bit
  analogReadResolution(12);

  // --- BARE METAL OPAMP SETUP ---
  // 1. Enable Clock for SYSCFG (Required to use OPAMPs)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // 2. Enable GPIO Clocks (A and B)
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

  // 3. Set Input Pins (PA1, PA7, PB0) to Analog Mode (11 in binary)
  GPIOA->MODER |= (3 << (1 * 2));  // PA1 (Phase V)
  GPIOA->MODER |= (3 << (7 * 2));  // PA7 (Phase W)
  GPIOB->MODER |= (3 << (0 * 2));  // PB0 (Phase U)

  // 4. Configure OPAMP1 (Phase V on PA1) -> Output PA2
  OPAMP1->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);

  // 5. Configure OPAMP2 (Phase W on PA7) -> Output PA6
  OPAMP2->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);

  // 6. Configure OPAMP3 (Phase U on PB0) -> Output PB1
  OPAMP3->CSR = (1 << 0) | (2 << 2) | (3 << 5) | (2 << 9) | (0 << 11);


  // --- TIMER 1 SETUP (20kHz) ---
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
  GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
  
  GPIOA->AFR[1] &= ~((0xF << 0) | (0xF << 4) | (0xF << 8));
  GPIOA->AFR[1] |= (6 << 0) | (6 << 4) | (6 << 8); 

  TIM1->PSC = 0;           
  TIM1->ARR = 8500 - 1;    
  
  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
  TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M);
  TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
  TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos);
  
  TIM1->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  TIM1->BDTR |= TIM_BDTR_MOE;

  // --- THE ARDUINO CORE INTERRUPT HOOK ---
  // We use the core's library to register our function to Timer 1 safely
  HardwareTimer *MyTim = new HardwareTimer(TIM1);
  MyTim->attachInterrupt(syncMeasurementISR);
  
  // Re-force the Update Interrupt Enable just in case the library cleared it
  TIM1->DIER |= TIM_DIER_UIE;
}

// --- THE INTERRUPT SERVICE ROUTINE (ISR) ---
void syncMeasurementISR() {
  // A static counter to slow down the measurements
  static int isr_divider = 0;
  isr_divider++;

  // Only run the slow analogRead() once every 200 PWM cycles (100 Hz)
  if (isr_divider >= 200) { 
    isr_divider = 0; // Reset counter

    // Grab the values
    int rawV = analogRead(PA2); 
    int rawW = analogRead(PA6); 
    int rawU = analogRead(PB1); 

    // Process math
    ampsV = ((rawV / 4095.0) * 3.3) / 0.16;
    ampsW = ((rawW / 4095.0) * 3.3) / 0.16;
    ampsU = ((rawU / 4095.0) * 3.3) / 0.16;
    
    newDataReady = true;
  }
}

// --- MAIN LOOP ---
void loop() {
  // 1. Print data if the ISR has updated the readings
  if (newDataReady) {
    newDataReady = false; 
    
    // We only print occasionally (roughly every ~40ms) to avoid lagging the serial port
    static int printCounter = 0;
    if (printCounter++ > 10) { 
      Serial.print("Phase_U:"); Serial.print(ampsU, 3); Serial.print(",");
      Serial.print("Phase_V:"); Serial.print(ampsV, 3); Serial.print(",");
      Serial.print("Phase_W:"); Serial.println(ampsW, 3);
      printCounter = 0;
    }
  }

  // 2. Execute the current 6-Step Commutation phase
  switch(step) {
    case 1:
      TIM1->CCR1 = pwmValue; // U High
      TIM1->CCR2 = 0;        // V Low
      TIM1->CCR3 = 0;        // W Low
      break;
    case 2:
      TIM1->CCR1 = pwmValue; // U High
      TIM1->CCR2 = pwmValue; // V High
      TIM1->CCR3 = 0;        // W Low
      break;
    case 3:
      TIM1->CCR1 = 0;        // U Low
      TIM1->CCR2 = pwmValue; // V High
      TIM1->CCR3 = 0;        // W Low
      break;
    case 4:
      TIM1->CCR1 = 0;        // U Low
      TIM1->CCR2 = pwmValue; // V High
      TIM1->CCR3 = pwmValue; // W High
      break;
    case 5:
      TIM1->CCR1 = 0;        // U Low
      TIM1->CCR2 = 0;        // V Low
      TIM1->CCR3 = pwmValue; // W High
      break;
    case 6:
      TIM1->CCR1 = pwmValue; // U High
      TIM1->CCR2 = 0;        // V Low
      TIM1->CCR3 = pwmValue; // W High
      break;
  }

  // 3. Move to the next step
  step++;
  if(step > 6) {
    step = 1; // Reset back to step 1 to complete the circle
  }

  // 4. Wait before taking the next step (This controls the physical RPM)
  delay(stepDelay); 
}