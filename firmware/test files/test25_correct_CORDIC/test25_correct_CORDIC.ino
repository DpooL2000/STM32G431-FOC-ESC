#include <Arduino.h>

// --- HARDWARE CORDIC INTERFACE ---
inline void get_Fast_Sin_Cos(float angle_rad, float* out_sin, float* out_cos) {
  // 1. Wrap angle strictly to [-PI, PI]
  while(angle_rad > PI) angle_rad -= 2.0f * PI;
  while(angle_rad < -PI) angle_rad += 2.0f * PI;

  // 2. Convert to q1.31 fixed-point
  int32_t angle_q31 = (int32_t)((angle_rad / PI) * 2147483648.0f);

  // 3. Write Angle to CORDIC
  CORDIC->WDATA = angle_q31;

  // 4. Read Results (Order: 1st read = Cos, 2nd read = Sin)
  int32_t res_cos = CORDIC->RDATA;
  int32_t res_sin = CORDIC->RDATA;

  *out_cos = (float)res_cos / 2147483648.0f;
  *out_sin = (float)res_sin / 2147483648.0f;
}

void setup() {
  Serial.begin(115200);
  while(!Serial); 
  
  // Enable CORDIC Clock
  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  
  // --- THE REAL FIX ---
  // Using official CMSIS Macros so we hit the exact correct bits:
  // CORDIC_CSR_PRECISION_Pos (Bit 4) -> Set to 6 (24 cycles)
  // CORDIC_CSR_NRES (Bit 18) -> Sets Number of Results to 2 (Cos and Sin)
  // FUNC defaults to 0 (Cosine/Sine)
  // NARGS defaults to 0 (1 Argument: Angle)
  CORDIC->CSR = (6 << CORDIC_CSR_PRECISION_Pos) | CORDIC_CSR_NRES; 

  Serial.println("\n--- CORDIC DEGREES TESTER (ACTUALLY FIXED) ---");
  Serial.println("Enter angle in DEGREES:");
}

void loop() {
  if (Serial.available() > 0) {
    float deg = Serial.parseFloat();
    while(Serial.available()) Serial.read(); 

    // Convert Degrees to Radians
    float rad = deg * (PI / 180.0f);

    float s_hw, c_hw;
    get_Fast_Sin_Cos(rad, &s_hw, &c_hw);
    
    float s_sw = sinf(rad);
    float c_sw = cosf(rad);

    Serial.print("\n[ DEGREES: "); Serial.print(deg, 2); Serial.println(" ]");
    Serial.println("----------------------------------------");
    Serial.print("SIN (HW): "); Serial.print(s_hw, 8); Serial.print(" | SW: "); Serial.print(s_sw, 8);
    Serial.print(" | ERR: "); Serial.println(abs(s_hw - s_sw), 12);
    
    Serial.print("COS (HW): "); Serial.print(c_hw, 8); Serial.print(" | SW: "); Serial.print(c_sw, 8);
    Serial.print(" | ERR: "); Serial.println(abs(c_hw - c_sw), 12);
    Serial.println("----------------------------------------");
  }
}