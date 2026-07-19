// Pins: PA8=U, PA9=V, PA10=W
const int pinU = PA8;
const int pinV = PA9;
const int pinW = PA10;

void setup() {
  pinMode(pinU, OUTPUT);
  pinMode(pinV, OUTPUT);
  pinMode(pinW, OUTPUT);

  // Initial Safety: All Low-Sides ON (Grounded)
  digitalWrite(pinU, HIGH);
  digitalWrite(pinV, HIGH);
  digitalWrite(pinW, HIGH);
}

void loop() {
  // --- DIRECT PATH: U (20% PWM) to V (GND) ---
  // Phase V must stay LOW-SIDE ON the whole time.
  // In our logic: STM32 HIGH = Low-Side ON.
  digitalWrite(pinV, HIGH);
  
  // Phase W stays LOW-SIDE ON (Safety/Ground)
  digitalWrite(pinW, HIGH);

  // --- PHASE U PWM LOOP (5kHz = 200us) ---
  // 20% of 200us = 40us HIGH SIDE ON (STM32 LOW)
  digitalWrite(pinU, LOW); 
  delayMicroseconds(10);
  
  // 80% of 200us = 160us LOW SIDE ON (STM32 HIGH)
  digitalWrite(pinU, HIGH);
  delayMicroseconds(190);
}