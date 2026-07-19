void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Hardware Hack Phase: Buffered DAC1_CH1 -> PA4");

  // =======================================================
  // 1. ENABLE CLOCKS (Using DAC1 this time!)
  // =======================================================
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_DAC1EN;

  // =======================================================
  // 2. CONFIGURE PA4
  // =======================================================
  // Set PA4 strictly to Analog Mode
  GPIOA->MODER |= (3 << GPIO_MODER_MODE4_Pos);

  // =======================================================
  // 3. CONFIGURE DAC1 CHANNEL 1 (Direct Output)
  // =======================================================
  DAC1->CR &= ~DAC_CR_EN1;             // Disable Channel 1 before config
  DAC1->MCR &= ~DAC_MCR_MODE1_Msk;     // Clear Mode Bits
  
  // By leaving the mode bits at 000, we select:
  // "DAC channel is connected to external pin with Buffer Enabled"
  // The internal buffer will provide the muscle to drive the AD8418s!
  
  DAC1->CR |= DAC_CR_EN1;              // Enable DAC1 Channel 1
  
  // 12-bit DAC: 4095 = 3.3V. 2048 = 1.65V.
  DAC1->DHR12R1 = 2048;                

  Serial.println("PA4 is now outputting a buffered 1.65V. Ready for the jumper wire.");
}

void loop() {
  delay(100);
}