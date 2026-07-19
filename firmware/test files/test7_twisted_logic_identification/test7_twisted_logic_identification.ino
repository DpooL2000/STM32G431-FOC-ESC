const int pinLowSide = PA10;  // Connects to IR2104 IN (via 6N137)
const int pinHighSide = PB9; // Connects to IR2104 SD (via PC817)

void setup() {
  Serial.begin(115200);
  pinMode(pinLowSide, OUTPUT);
  pinMode(pinHighSide, OUTPUT);

  // Safety: Start with both OFF
  digitalWrite(pinLowSide, LOW);
  digitalWrite(pinHighSide, LOW);
  
  Serial.println("Starting Single Bridge Test @ 1kHz");
  Serial.println("Software Dead-Time: 100us");
}

void loop() {
  // We will run a manual PWM loop to ensure strict timing
  // Total period = 1ms (1000us)
  
  // 1. HIGH-SIDE PULSE (20% Duty)
  // Logic: SD=HIGH, IN=LOW
  digitalWrite(pinLowSide, LOW);   // Ensure Low is OFF
  delayMicroseconds(100);          // DEAD-TIME
  digitalWrite(pinHighSide, HIGH); // Turn High ON
  delayMicroseconds(200);          // Pulse for 200us
  digitalWrite(pinHighSide, LOW);  // Turn High OFF
  
  // 2. WAIT FOR LOW-SIDE (DEAD-TIME)
  delayMicroseconds(100);          // DEAD-TIME
  
  // 3. LOW-SIDE PULSE (20% Duty)
  // Logic: SD=LOW, IN=HIGH (6N137 inverts, so STM32 must go HIGH for IR2104 to see LOW)
  // WAIT - Based on your logic map: STM32 HIGH at IN = Low-Side ON.
  digitalWrite(pinLowSide, HIGH);  
  delayMicroseconds(200);
  digitalWrite(pinLowSide, LOW);
  
  // 4. REMAINING IDLE TIME (to complete 1000us cycle)
  // Total used: 100+200+100+200 = 600us. Remaining: 400us.
  delayMicroseconds(400);
}