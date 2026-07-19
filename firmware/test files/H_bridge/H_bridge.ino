/* * H-Bridge Concept Test (Scope Only)
 * Board: STM32G431CBU6
 * Pins: PA8 (Bridge A), PA10 (Bridge B), PB9 (Enable), PC6 (LED)
 */

const int phaseA = PA8;
const int phaseB = PA10;
const int drv_en = PB9;
const int led    = PC6;

// Using a 10% Duty Cycle (410 out of 4095) for a very "clean" scope signal
const int testDuty = 410; 

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  pinMode(drv_en, OUTPUT);
  
  // Keep system disabled during boot
  digitalWrite(drv_en, LOW);
  digitalWrite(led, LOW);

  // G431 High-Speed PWM Config
  analogWriteFrequency(20000); // 20kHz switching
  analogWriteResolution(12);   // 0-4095 range

  delay(1000);
  Serial.println("--- H-BRIDGE CONCEPT TEST STARTING ---");
  
  digitalWrite(drv_en, HIGH); // Enable IR2104 drivers
  digitalWrite(led, HIGH);
}

void loop() {
  // STATE 1: Current flows from Phase A to Phase B
  // Phase A High-Side pulses, Phase B stays LOW (Low-side MOSFET ON)
  Serial.println("Direction: A -> B");
  analogWrite(phaseA, testDuty); 
  analogWrite(phaseB, 0);         
  delay(1000); 

  // STATE 2: Dead-time / Idle (Safety)
  analogWrite(phaseA, 0);
  analogWrite(phaseB, 0);
  delay(200);

  // STATE 3: Current flows from Phase B to Phase A
  // Phase B High-Side pulses, Phase A stays LOW (Low-side MOSFET ON)
  Serial.println("Direction: B -> A");
  analogWrite(phaseA, 0);
  analogWrite(phaseB, testDuty);
  delay(1000);

  // STATE 4: Dead-time / Idle
  analogWrite(phaseA, 0);
  analogWrite(phaseB, 0);
  delay(200);
}