// --- CK's Rigged Logic Pin Mapping ---
const int pinU_IN = PA8;  // Phase U (via Inverting 6N137)
const int pinV_IN = PA9;  // Phase V (via Inverting 6N137)
const int pinW_IN = PA10; // Phase W (via Inverting 6N137)
const int pinSD_All = PB9;// All SD Pins (via Non-Inverting PC817)

void setup() {
  Serial.begin(115200);
  pinMode(pinU_IN, OUTPUT);
  pinMode(pinV_IN, OUTPUT);
  pinMode(pinW_IN, OUTPUT);
  pinMode(pinSD_All, OUTPUT);

  // --- SAFE START (Mismatch State) ---
  // PAx=LOW -> IN=5V. PB9=LOW -> SD=0V. 
  // Result: IN(5V) != SD(0V) -> Both Gates OFF.
  digitalWrite(pinU_IN, LOW);
  digitalWrite(pinV_IN, LOW);
  digitalWrite(pinW_IN, LOW);
  digitalWrite(pinSD_All, LOW);
  
  Serial.println("Starting 1kHz Phase U Test (20% Duty)");
  Serial.println("Phases V & W grounded (0% Duty)");
}

void loop() {
  // Total Cycle = 1000us (1kHz)
  
  // ==========================================
  // STATE 1: PHASE U HIGH-SIDE PULSE (200us)
  // ==========================================
  // We want U HO ON, but V & W MUST be OFF so we don't shoot 12V into ground.
  // U Target: IN=5, SD=5 (HO ON) -> PA8=LOW, PB9=HIGH
  // V/W Target: IN=0, SD=5 (MISMATCH = OFF) -> PA9/PA10=HIGH, PB9=HIGH
  digitalWrite(pinU_IN, LOW);  
  digitalWrite(pinV_IN, HIGH); 
  digitalWrite(pinW_IN, HIGH); 
  digitalWrite(pinSD_All, HIGH); 
  delayMicroseconds(200);

  // ==========================================
  // STATE 2: DEAD-TIME (100us)
  // ==========================================
  // Return to the Safe Mismatch State (IN=5, SD=0) while the slow PC817 turns off.
  digitalWrite(pinU_IN, LOW);
  digitalWrite(pinV_IN, LOW);
  digitalWrite(pinW_IN, LOW);
  digitalWrite(pinSD_All, LOW);
  delayMicroseconds(100);

  // ==========================================
  // STATE 3: ALL LOW-SIDES ON (600us)
  // ==========================================
  // We want U, V, and W LO ON to recharge boot caps and provide GND path.
  // Target: IN=0, SD=0 (LO ON) -> PA8/9/10=HIGH, PB9=LOW
  digitalWrite(pinU_IN, HIGH);
  digitalWrite(pinV_IN, HIGH);
  digitalWrite(pinW_IN, HIGH);
  digitalWrite(pinSD_All, LOW);
  delayMicroseconds(600);

  // ==========================================
  // STATE 4: DEAD-TIME (100us)
  // ==========================================
  // Return to Safe Mismatch State before the loop restarts.
  digitalWrite(pinU_IN, LOW);
  digitalWrite(pinV_IN, LOW);
  digitalWrite(pinW_IN, LOW);
  digitalWrite(pinSD_All, LOW);
  delayMicroseconds(100);
}