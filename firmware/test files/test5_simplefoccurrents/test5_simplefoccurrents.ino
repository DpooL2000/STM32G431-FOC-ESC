/**
 * Testing example code for the Inline current sensing class
*/
#include <SimpleFOC.h>

// current sensor
// shunt resistor value: 0.002 Ohms (2mOhm)
// gain value: 50.0f
// pins phase A,B, (C optional)
InlineCurrentSense current_sense = InlineCurrentSense(0.002f, 50.0f, PA1, PA7, PB0);

void setup() {
  // =======================================================
  // 1. THE 1.65V HARDWARE HACK (PA4 outputs, PA6 receives)
  // MUST EXECUTE BEFORE ANY SENSOR INITIALIZATION
  // =======================================================
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_DAC1EN;
  
  // Set PA4 and PA6 to Analog Mode safely to prevent shorts
  GPIOA->MODER |= (3 << GPIO_MODER_MODE4_Pos) | (3 << GPIO_MODER_MODE6_Pos);

  DAC1->CR &= ~DAC_CR_EN1;             
  DAC1->MCR &= ~DAC_MCR_MODE1_Msk;     // Buffer Enabled Mode
  DAC1->CR |= DAC_CR_EN1;              
  DAC1->DHR12R1 = 2048;                // Output 1.65V to PA4
  // =======================================================

  // Tell SimpleFOC to use the full 12-bit resolution of the STM32G4
  analogReadResolution(12);

  // use monitoring with serial 
  Serial.begin(115200);
  
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialise the current sensing
  if(!current_sense.init()){
    Serial.println("Current sense init failed.");
    return;
  }

  // Only uncomment this if your Phase B readings are backwards under load
  // current_sense.gain_b *= -1;
  
  Serial.println("Current sense ready.");
}

void loop() {
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();

  // ONLY print every 20 milliseconds!
  static unsigned long last_plot_time = 0;
  if (millis() - last_plot_time > 2) { 
    Serial.print(currents.a * 1000); Serial.print("\t");
    Serial.print(currents.b * 1000); Serial.print("\t");
    Serial.print(currents.c * 1000); Serial.print("\t");
    Serial.println(current_magnitude * 1000); 
    last_plot_time = millis();
  }
}