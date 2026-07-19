#include <SimpleFOC.h>

// ==========================================
// 1. PIN DEFINITIONS (STM32G431 to 6N137 to IR2101)
// ==========================================
const int PhaseA_High = PA8;
const int PhaseA_Low  = PB13;
const int PhaseB_High = PA9;
const int PhaseB_Low  = PB14;
const int PhaseC_High = PA10;
const int PhaseC_Low  = PB15;

// BEMF Sensing Pins (11:1 Voltage Divider)
const int BEMF_A = PA0;
const int BEMF_B = PA3;
const int BEMF_C = PB1;

// ==========================================
// 2. MOTOR & DRIVER SETUP
// ==========================================
BLDCMotor motor = BLDCMotor(7); 
BLDCDriver6PWM driver = BLDCDriver6PWM(PhaseA_High, PhaseA_Low, PhaseB_High, PhaseB_Low, PhaseC_High, PhaseC_Low);

// ==========================================
// 3. CURRENT SENSING SETUP (AD8418 Inline)
// ==========================================
InlineCurrentSense current_sense = InlineCurrentSense(0.002, 50.0, PA1, PA7, PB0);

// ==========================================
// 4. OSCILLOSCOPE FILTER VARIABLES
// ==========================================
float filter_alpha = 0.05; 
float filtered_A = 0.0, filtered_B = 0.0, filtered_C = 0.0;
float target_velocity = 0.0;

void setup() {
  // --------------------------------------------------------
  // CRITICAL SAFETY BLOCK: Lock gates closed before anything else
  // --------------------------------------------------------
  digitalWrite(PhaseA_High, HIGH);
  digitalWrite(PhaseA_Low,  HIGH);
  digitalWrite(PhaseB_High, HIGH);
  digitalWrite(PhaseB_Low,  HIGH);
  digitalWrite(PhaseC_High, HIGH);
  digitalWrite(PhaseC_Low,  HIGH);

  pinMode(PhaseA_High, OUTPUT);
  pinMode(PhaseA_Low,  OUTPUT);
  pinMode(PhaseB_High, OUTPUT);
  pinMode(PhaseB_Low,  OUTPUT);
  pinMode(PhaseC_High, OUTPUT);
  pinMode(PhaseC_Low,  OUTPUT);
  delay(100); 
  
  Serial.begin(115200);
  analogReadResolution(12);

  // --------------------------------------------------------
  // BARE-METAL 1.65V REFERENCE (DAC3 -> OPAMP2 -> PA6)
  // --------------------------------------------------------
  RCC->AHB2ENR |= RCC_AHB2ENR_DAC3EN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  DAC3->CR &= ~DAC_CR_EN1; 
  DAC3->MCR &= ~DAC_MCR_MODE1_Msk; 
  DAC3->MCR |= (3 << DAC_MCR_MODE1_Pos); 
  DAC3->CR |= DAC_CR_EN1; 
  DAC3->DHR12R1 = 2048;   

  OPAMP2->CSR = 0; 
  OPAMP2->CSR |= (3 << OPAMP_CSR_VMSEL_Pos); 
  OPAMP2->CSR |= (2 << OPAMP_CSR_VPSEL_Pos); 
  OPAMP2->CSR |= OPAMP_CSR_OPAMPINTEN;       
  delay(1); 

  // --------------------------------------------------------
  // DRIVER CONFIGURATION
  // --------------------------------------------------------
  driver.voltage_power_supply = 12.0;
  driver.voltage_limit = 12.0;

  // IMPORTANT: Hardware inversion handled by build_opt.h flag!
  driver.dead_zone = 0.05; 

  driver.init();
  motor.linkDriver(&driver);

  // --------------------------------------------------------
  // CURRENT SENSE CONFIGURATION
  // --------------------------------------------------------
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // --------------------------------------------------------
  // MOTOR CONFIGURATION & LIMITS (Sensorless / Open Loop)
  // --------------------------------------------------------
  motor.controller = MotionControlType::velocity_openloop;
  
  // THE ANTI-EXPLOSION LIMIT: Never exceed 2V average on phases
  motor.voltage_limit = 2.0; 
  
  motor.init();
}

void loop() {
  motor.move(target_velocity);

  // SILENT SERIAL PARSING (Fixes the Arduino IDE Newline glitch)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Strips hidden characters that caused the freeze
    if (input.length() > 0) {
      target_velocity = input.toFloat();
      Serial.print("set to:");Serial.println(target_velocity);
    }
  }

  // Read BEMF for the Serial Plotter
  int raw_A = analogRead(BEMF_A);
  int raw_B = analogRead(BEMF_B);
  int raw_C = analogRead(BEMF_C);

  // Convert raw 12-bit ADC data to actual phase voltage (11:1 divider)
  float volts_A = (raw_A / 4095.0) * 3.3 * 11.0;
  float volts_B = (raw_B / 4095.0) * 3.3 * 11.0;
  float volts_C = (raw_C / 4095.0) * 3.3 * 11.0;

  // Apply Software Low-Pass Filter
  filtered_A = (filter_alpha * volts_A) + ((1.0 - filter_alpha) * filtered_A);
  filtered_B = (filter_alpha * volts_B) + ((1.0 - filter_alpha) * filtered_B);
  filtered_C = (filter_alpha * volts_C) + ((1.0 - filter_alpha) * filtered_C);

  // Print strictly to Serial Plotter format every 20ms
  static unsigned long last_plot_time = 0;
  if (millis() - last_plot_time > 20) { 
    Serial.print("Phase_A:"); Serial.print(filtered_A); Serial.print(",");
    Serial.print("Phase_B:"); Serial.print(filtered_B); Serial.print(",");
    Serial.print("Phase_C:"); Serial.println(filtered_C);
    last_plot_time = millis();
  }
}