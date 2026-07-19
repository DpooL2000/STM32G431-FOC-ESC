#include <SimpleFOC.h>

// --- HARDWARE & CALIBRATION ---
const int BEMF_A = PA0, BEMF_B = PA3, BEMF_C = PB1;
const float VOLTAGE_LIMIT = 4.0f; 
const float POWER_SUPPLY = 12.0f;
const float BEMF_SCL = (3.3f / 4095.0f) * 11.0f;
const float SQRT3_2 = 0.866025f;

// --- SENSORS & MATH ---
InlineCurrentSense current_sense = InlineCurrentSense(0.002f, 50.0f, PA1, PA7, PB0);

// --- GLOBAL STATE VARIABLES ---
float electrical_angle = 0.0;
float estimated_velocity = 0.0;
unsigned long last_time = 0;
unsigned long last_crossing_time = 0;

// BEMF Observers
float last_zcA = 0, last_zcB = 0, last_zcC = 0;
bool sensorless_synced = false;

// FOC Targets & PID
float target_Iq = 0.0, target_Id = 0.0;
float integral_d = 0, integral_q = 0;
float Kp = 0.15f, Ki = 1.5f; 

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_Iq, cmd); }

// --- BARE-METAL TIM1 (Safe 0xFFF Polarity) ---
void configureTIM1() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  GPIOA->MODER = (GPIOA->MODER & ~0x3F0000) | 0x2A0000; 
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFF) | 0x666;
  GPIOB->MODER = (GPIOB->MODER & ~0xFC000000) | 0xA8000000; 
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFF00000) | 0x46600000; 

  TIM1->CR1 = 0x20; TIM1->PSC = 0; TIM1->ARR = 4250;
  TIM1->CCMR1 = 0x6868; TIM1->CCMR2 = 0x68;
  TIM1->CCER = 0xFFF; // Active-Low for 6N137
  TIM1->BDTR = 0x8064; 
  TIM1->CR2 = 0xF3F; 
  TIM1->EGR |= 1; TIM1->CR1 |= 1; 
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  
  // 1.65V Hack + Clock Enable
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_DAC1EN);
  GPIOA->MODER |= (3 << 8) | (3 << 12); 
  DAC1->CR |= 1; DAC1->DHR12R1 = 2048;

  configureTIM1();
  current_sense.init();
  
  command.add('T', doTarget, "Target Iq");
  
  last_time = micros();
  last_crossing_time = micros();
  Serial.println("Sensorless FOC Engine Ready.");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_time) * 1e-6f;
  if(dt <= 0 || dt > 0.1f) dt = 1e-4f;
  last_time = now;

 // =================================================================
  // 1. THE 6-SECTOR ZERO-CROSSING OBSERVER (FULL SYNC)
  // =================================================================
  float vA = analogRead(BEMF_A), vB = analogRead(BEMF_B), vC = analogRead(BEMF_C);
  float neutral = (vA + vB + vC) / 3.0f;
  float zcA = vA - neutral;
  float zcB = vB - neutral;
  float zcC = vC - neutral;

  bool triggered = false;
  float snap_angle = 0;

  // Sector 1: Phase A Falling (Angle 0)
  if (last_zcA > 0 && zcA <= 0) { snap_angle = 0.0f; triggered = true; }
  
  // Sector 2: Phase C Rising (Angle 60 deg)
  else if (last_zcC < 0 && zcC >= 0) { snap_angle = _PI / 3.0f; triggered = true; }
  
  // Sector 3: Phase B Falling (Angle 120 deg)
  else if (last_zcB > 0 && zcB <= 0) { snap_angle = _2PI / 3.0f; triggered = true; }
  
  // Sector 4: Phase A Rising (Angle 180 deg)
  else if (last_zcA < 0 && zcA >= 0) { snap_angle = _PI; triggered = true; }
  
  // Sector 5: Phase C Falling (Angle 240 deg)
  else if (last_zcC > 0 && zcC <= 0) { snap_angle = 4.0f * _PI / 3.0f; triggered = true; }
  
  // Sector 6: Phase B Rising (Angle 300 deg)
  else if (last_zcB < 0 && zcB >= 0) { snap_angle = 5.0f * _PI / 3.0f; triggered = true; }

  if (triggered) {
    unsigned long crossing_dt = now - last_crossing_time;
    if (crossing_dt > 100) { // Simple debounce for high-frequency noise
        float sector_time = crossing_dt * 1e-6f;
        
        // Calculate velocity based on 60 degrees (PI/3 rad) of travel
        estimated_velocity = (_PI / 3.0f) / sector_time;
        
        // Snap the angle to the physical reality
        electrical_angle = snap_angle;
        
        last_crossing_time = now;
        sensorless_synced = true;
    }
  }

  // Save states for next loop
  last_zcA = zcA; last_zcB = zcB; last_zcC = zcC;

  last_zcA = zcA; last_zcB = zcB; last_zcC = zcC;

  // Integration between crossings
  if (sensorless_synced) {
    electrical_angle += estimated_velocity * dt;
  } else {
    // LAME KICKSTART: Slow ramp until BEMF is readable
    electrical_angle += 5.0f * dt; 
  }
  
  if (electrical_angle > _2PI) electrical_angle -= _2PI;

  // =================================================================
  // 2. FORWARD FOC (Current Sensing)
  // =================================================================
  PhaseCurrent_s I = current_sense.getPhaseCurrents();
  float s = _sin(electrical_angle), c = _cos(electrical_angle);
  
  float I_alpha = I.a;
  float I_beta = (I.a + 2.0f * I.b) * 0.57735f;
  float Id = I_alpha * c + I_beta * s;
  float Iq = -I_alpha * s + I_beta * c;

  // =================================================================
  // 3. PI CURRENT CONTROL
  // =================================================================
  float err_d = target_Id - Id;
  float err_q = target_Iq - Iq;
  
  integral_d = _constrain(integral_d + err_d * Ki * dt, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  integral_q = _constrain(integral_q + err_q * Ki * dt, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

  float Vd = _constrain(err_d * Kp + integral_d, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  float Vq = _constrain(err_q * Kp + integral_q, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

  // =================================================================
  // 4. INVERSE FOC & ACTUATE
  // =================================================================
  float V_alpha = Vd * c - Vq * s;
  float V_beta  = Vd * s + Vq * c;
  
  float Va = V_alpha;
  float Vb = -0.5f * V_alpha + SQRT3_2 * V_beta;
  float Vc = -0.5f * V_alpha - SQRT3_2 * V_beta;

  TIM1->CCR1 = (uint16_t)(_constrain((Va/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);
  TIM1->CCR2 = (uint16_t)(_constrain((Vb/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);
  TIM1->CCR3 = (uint16_t)(_constrain((Vc/POWER_SUPPLY)+0.5f, 0.05f, 0.95f) * 4250);

  // Plotter: Check if Iq stays steady while BEMF oscillates
  static unsigned long lp = 0;
  if (millis() - lp > 20) {
    Serial.print("Iq:"); Serial.print(Iq); Serial.print(",");
    Serial.print("Ia:"); Serial.print(I.a); Serial.print(",");
    Serial.print("Ib:"); Serial.print(I.b); Serial.print(",");
    Serial.print("Ic:"); Serial.print(I.c); Serial.print(",");
    Serial.print("BEMF_A:"); Serial.println(zcA);
    lp = millis();
  }
  
  command.run();
}