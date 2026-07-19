#include <SimpleFOC.h>

const int PhaseA_High = PA8;
const int PhaseA_Low = PB13;
const int PhaseB_High = PA9;
const int PhaseB_Low = PB14;
const int PhaseC_High = PA10;
const int PhaseC_Low = PB15;

const int BEMF_A = PA0;
const int BEMF_B = PA3;
const int BEMF_C = PB1;

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(PhaseA_High, PhaseA_Low, PhaseB_High, PhaseB_Low, PhaseC_High, PhaseC_Low);
InlineCurrentSense current_sense = InlineCurrentSense(0.002, 50.0, PA1, PA7, PB0);

float filter_alpha = 0.05;
float filtered_A = 0.0, filtered_B = 0.0, filtered_C = 0.0;
float target_velocity = 0.0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // 1. WAKE UP THE 1.65V OPAMP REFERENCE (Does not interfere with TIM1)
  RCC->AHB2ENR |= RCC_AHB2ENR_DAC3EN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  DAC3->CR &= ~DAC_CR_EN1;
  DAC3->MCR &= ~DAC_MCR_MODE1_Msk;
  DAC3->MCR |= (3 << DAC_MCR_MODE1_Pos);
  DAC3->CR |= DAC_CR_EN1;
  DAC3->DHR12R1 = 2048;
  OPAMP2->CSR = 0;
  OPAMP2->CSR |= (3 << OPAMP_CSR_VMSEL_Pos) | (2 << OPAMP_CSR_VPSEL_Pos) | OPAMP_CSR_OPAMPINTEN;
  delay(1);

  // 2. PURE SIMPLEFOC INITIALIZATION
  driver.voltage_power_supply = 12.0;
  driver.dead_zone = 0.05;

  // This function will now successfully grab the pins and start the 20kHz PWM
  driver.init();
  motor.linkDriver(&driver);

  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 2.0;
  motor.init();

  Serial.println("System Ready. Timer is actively pulsing.");
  Serial.println(">>> YOU MAY NOW TURN ON THE 12V MAIN POWER <<<");
}

void loop() {
  motor.move(target_velocity);

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      target_velocity = input.toFloat();
      Serial.print("Target set to: ");
      Serial.println(target_velocity);
    }
  }

  // BEMF Scope Plotting
  int raw_A = analogRead(BEMF_A);
  int raw_B = analogRead(BEMF_B);
  int raw_C = analogRead(BEMF_C);

  float volts_A = (raw_A / 4095.0) * 3.3 * 11.0;
  float volts_B = (raw_B / 4095.0) * 3.3 * 11.0;
  float volts_C = (raw_C / 4095.0) * 3.3 * 11.0;

  filtered_A = (filter_alpha * volts_A) + ((1.0 - filter_alpha) * filtered_A);
  filtered_B = (filter_alpha * volts_B) + ((1.0 - filter_alpha) * filtered_B);
  filtered_C = (filter_alpha * volts_C) + ((1.0 - filter_alpha) * filtered_C);

  static unsigned long last_plot_time = 0;
  if (millis() - last_plot_time > 20) {
    Serial.print("Phase_A:");
    Serial.print(filtered_A);
    Serial.print(",");
    Serial.print("Phase_B:");
    Serial.print(filtered_B);
    Serial.print(",");
    Serial.print("Phase_C:");
    Serial.println(filtered_C);
    last_plot_time = millis();
  }
}