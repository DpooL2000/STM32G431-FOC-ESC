
// =================================================================
// SELF-COMMISSIONING V7.0
// Target: WeAct STM32G431 | A2212 2200KV 7pp
//
// CHANGES FROM V6:
//
// Lambda — removed V_BREAK entirely. Root cause of all overcurrent:
//   V_BREAK=2.5V into R=92mOhm = 27A before back-EMF builds.
//   The AD8418 on 3.3V supply saturates well below that.
//
//   New approach: correct V/f law with k = lambda_KV.
//   V_mag = V_BASE + lambda_KV * omega_elec
//   Since back-EMF = lambda * omega_elec, and k ≈ lambda,
//   the extra voltage above V_BASE exactly cancels the back-EMF.
//   Result: I_ss ≈ V_BASE/R = 5A throughout the ENTIRE ramp.
//   No transient spikes. No break voltage. No overcurrent.
//
//   V_BASE = I_target * R = 5A * 0.092 = 0.46V
//   k = 0.000620 V/(rad/s) = lambda_KV for 2200KV 7pp motor
//   Max V at 1500 rad/s: 0.46 + 0.62*1.5 = 1.39V — totally safe.
//
//   Stall detection: spinning I ~ 5A, stalled I ~ 15A. Threshold 10A.
//   Check fires 500ms after reaching target speed (no mode switching needed).
// =================================================================

#include <Arduino.h>
#include <HardwareSerial.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

const float POWER_SUPPLY           = 12.0f;
const float LOOP_DT                = 50e-6f;
const float NOMINAL_AMPS_PER_COUNT = (3.3f / 4095.0f) / (0.002f * 20.0f);

float offset_U = 2048.0f;
float offset_V = 2048.0f;

// ----------------------------------------------------------------
// COMMISSIONING PARAMETERS
// ----------------------------------------------------------------
const float    R_TEST_VOLTAGE  = 0.5f;
const uint32_t R_SETTLE_TICKS  = 4000;
const uint32_t R_AVERAGE_TICKS = 2000;
const uint32_t R_DECAY_TICKS   = 200;

const float    L_INJECT_V      = 0.4f;
const float    L_FREQ_LO       = 200.0f;
const float    L_FREQ_HI       = 800.0f;
const uint32_t L_SETTLE_CYCLES = 5;
const uint32_t L_N_CYCLES      = 20;

// Lambda V/f law: V_mag = LAMBDA_V_BASE + LAMBDA_V_PER_RADS * omega_elec
// LAMBDA_V_PER_RADS = lambda_KV keeps current constant throughout ramp.
// LAMBDA_V_BASE = I_TARGET * R_phase sets the steady-state spin current.
// For R=92mOhm, I_TARGET=5A: V_BASE = 0.46V
// Do NOT increase V_BASE above I_SAFE_SPIN * R without checking ADC range.
const float    LAMBDA_I_TARGET      = 5.0f;    // A — spin current throughout ramp
const float    LAMBDA_V_BASE        = 0.46f;   // V = I_TARGET * R (92mOhm)
const float    LAMBDA_V_PER_RADS    = 0.000620f; // V/(rad/s) = lambda_KV for 2200KV p=7
const float    LAMBDA_TARGET_SPEED  = 1500.0f; // rad/s electrical
const float    LAMBDA_ACCEL         = 100.0f;  // rad/s^2 — gentle, motor tracks easily
const float    LAMBDA_LPF_TAU       = 0.005f;
const uint32_t LAMBDA_SETTLE_TICKS  = 10000;   // 500ms at speed before stall check
const uint32_t LAMBDA_MEASURE_TICKS = 40000;   // 2s measurement
// Spinning: I ~ I_TARGET ~ 5A. Stalled: I ~ V_at_speed/R ~ 15A. Threshold = 10A.
const float    I_STALL_THRESHOLD    = 10.0f;

const float    CURRENT_LIMIT_A      = 40.0f;

// ----------------------------------------------------------------
// HARDWARE
// ----------------------------------------------------------------
void hardware_lockdown() {
  pinMode(PA8,OUTPUT);pinMode(PA9,OUTPUT);pinMode(PA10,OUTPUT);
  pinMode(PB13,OUTPUT);pinMode(PB14,OUTPUT);pinMode(PB15,OUTPUT);
  digitalWrite(PA8,DRIVER_OFF);digitalWrite(PB13,DRIVER_OFF);
  digitalWrite(PA9,DRIVER_OFF);digitalWrite(PB14,DRIVER_OFF);
  digitalWrite(PA10,DRIVER_OFF);digitalWrite(PB15,DRIVER_OFF);
}

void configureTIM1() {
  RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;
  GPIOA->MODER=(GPIOA->MODER&~0x3F0000)|0x2A0000;
  GPIOA->AFR[1]=(GPIOA->AFR[1]&~0xFFF)|0x666;
  GPIOB->MODER=(GPIOB->MODER&~0xFC000000)|0xA8000000;
  GPIOB->AFR[1]=(GPIOB->AFR[1]&~0xFFF00000)|0x46600000;
  TIM1->CR1=0x20;TIM1->PSC=0;TIM1->ARR=4250;
  TIM1->CCMR1=0x6868;TIM1->CCMR2=0x6868;
  TIM1->CCR1=2125;TIM1->CCR2=2125;TIM1->CCR3=2125;TIM1->CCR4=4240;
  TIM1->CCER=0xFFF;TIM1->BDTR=0x80A0;
  TIM1->CR2=0xF3F;TIM1->CR2&=~TIM_CR2_MMS_Msk;
  TIM1->CR2|=(0x7<<TIM_CR2_MMS_Pos);
  TIM1->EGR|=1;TIM1->CR1|=1;
}

void setup_bare_metal_ADCs() {
  RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR|=RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN|RCC_AHB2ENR_DAC3EN|RCC_AHB2ENR_ADC12EN;
  GPIOA->MODER|=(3<<2)|(3<<4)|(3<<8)|(3<<12)|(3<<14);
  GPIOB->MODER|=(3<<0);
  GPIOA->PUPDR&=~((3<<4)|(3<<8)|(3<<12));
  DAC3->CR&=~DAC_CR_EN1;DAC3->MCR&=~DAC_MCR_MODE1_Msk;
  DAC3->MCR|=(3<<DAC_MCR_MODE1_Pos);
  DAC3->CR|=DAC_CR_EN1;DAC3->DHR12R1=2048;
  OPAMP1->CSR=(3<<5)|(3<<2);__asm__("nop");__asm__("nop");OPAMP1->CSR|=(1<<0);
  ADC12_COMMON->CCR&=~(3<<16);ADC12_COMMON->CCR|=(2<<16);
  ADC1->CR&=~ADC_CR_DEEPPWD;ADC2->CR&=~ADC_CR_DEEPPWD;delay(1);
  ADC1->CR|=ADC_CR_ADVREGEN;ADC2->CR|=ADC_CR_ADVREGEN;delay(1);
  ADC1->CR&=~ADC_CR_ADCALDIF;ADC1->CR|=ADC_CR_ADCAL;while(ADC1->CR&ADC_CR_ADCAL);
  ADC2->CR&=~ADC_CR_ADCALDIF;ADC2->CR|=ADC_CR_ADCAL;while(ADC2->CR&ADC_CR_ADCAL);delay(1);
  ADC1->ISR|=ADC_ISR_ADRDY;ADC1->CR|=ADC_CR_ADEN;while(!(ADC1->ISR&ADC_ISR_ADRDY));
  ADC2->ISR|=ADC_ISR_ADRDY;ADC2->CR|=ADC_CR_ADEN;while(!(ADC2->ISR&ADC_ISR_ADRDY));
  ADC1->SMPR1|=(4<<6);ADC1->SMPR2|=(4<<15);ADC2->SMPR1|=(4<<12);
  ADC1->JSQR=(1<<0)|(1<<7)|(15<<9)|(2<<15);
  ADC2->JSQR=(0<<0)|(1<<7)|(4<<9);
  ADC1->CR|=ADC_CR_JADSTART;ADC2->CR|=ADC_CR_JADSTART;
}

void setInverterVoltages(float va, float vb) {
  float Va=va, Vb=-0.5f*va+0.8660254f*vb, Vc=-0.5f*va-0.8660254f*vb;
  float vc=-(fmaxf(Va,fmaxf(Vb,Vc))+fminf(Va,fminf(Vb,Vc)))*0.5f;
  TIM1->CCR1=(uint16_t)(constrain(((Va+vc)/POWER_SUPPLY)+0.5f,0.02f,0.98f)*4250.0f);
  TIM1->CCR2=(uint16_t)(constrain(((Vb+vc)/POWER_SUPPLY)+0.5f,0.02f,0.98f)*4250.0f);
  TIM1->CCR3=(uint16_t)(constrain(((Vc+vc)/POWER_SUPPLY)+0.5f,0.02f,0.98f)*4250.0f);
  TIM1->CCER=0xFFF;
}

inline void get_Fast_Sin_Cos(float a, float* s, float* c) {
  while(a>_PI)a-=_2PI;while(a<-_PI)a+=_2PI;
  CORDIC->WDATA=(int32_t)((a/_PI)*2147483648.0f);
  int32_t rc=CORDIC->RDATA,rs=CORDIC->RDATA;
  *c=(float)rc/2147483648.0f;*s=(float)rs/2147483648.0f;
}

inline void readCurrents(float &Ia, float &Ib) {
  float IU=(float)((int32_t)ADC1->JDR1-(int)offset_U)*NOMINAL_AMPS_PER_COUNT;
  float IW=(float)((int32_t)ADC2->JDR1-(int)offset_V)*NOMINAL_AMPS_PER_COUNT;
  Ia=IU; Ib=-(IU+2.0f*IW)*0.577350269f;
}

inline void waitTick(){while(!(TIM1->SR&TIM_SR_UIF));TIM1->SR&=~TIM_SR_UIF;}

bool trip(float Ia, float Ib, const char* ctx) {
  float m=sqrtf(Ia*Ia+Ib*Ib);
  if(m>CURRENT_LIMIT_A){
    setInverterVoltages(0,0);hardware_lockdown();
    Serial.print("\n!!! TRIP [");Serial.print(ctx);
    Serial.print("] |I|=");Serial.print(m,2);Serial.println("A");
    return true;
  }
  return false;
}

// ================================================================
// PHASE 1 — RESISTANCE
// ================================================================
float measure_R() {
  Serial.println("\n--- Phase 1: Resistance ---");
  float Ia,Ib;
  setInverterVoltages(R_TEST_VOLTAGE,0);
  for(uint32_t i=0;i<R_SETTLE_TICKS;i++){waitTick();readCurrents(Ia,Ib);if(trip(Ia,Ib,"R"))return -1;}
  double sum=0;
  for(uint32_t i=0;i<R_AVERAGE_TICKS;i++){waitTick();readCurrents(Ia,Ib);if(trip(Ia,Ib,"R"))return -1;sum+=Ia;}
  setInverterVoltages(0,0);
  for(uint32_t i=0;i<R_DECAY_TICKS;i++) waitTick();
  float I_avg=(float)(sum/R_AVERAGE_TICKS);
  float R=(R_TEST_VOLTAGE*(2.0f/3.0f))/I_avg;
  Serial.print("  I_avg=");Serial.print(I_avg,4);Serial.print("A  R=");Serial.print(R*1000,3);Serial.println(" mOhm");
  return R;
}

// ================================================================
// PHASE 2 — INDUCTANCE (lock-in impedance at two frequencies)
// ================================================================
float measure_L_at_freq(float freq, const char* tag) {
  uint32_t tpc=(uint32_t)(1.0f/(freq*LOOP_DT)+0.5f);
  float da=_2PI*freq*LOOP_DT, ang=0;
  float Ia,Ib;
  for(uint32_t t=0;t<L_SETTLE_CYCLES*tpc;t++){
    waitTick();setInverterVoltages(L_INJECT_V*sinf(ang),0);
    readCurrents(Ia,Ib);if(trip(Ia,Ib,"L"))return -1;
    ang+=da;if(ang>_2PI)ang-=_2PI;
  }
  double sIs=0,sIc=0; uint32_t N=L_N_CYCLES*tpc;
  for(uint32_t t=0;t<N;t++){
    waitTick();
    float s=sinf(ang),c=cosf(ang);
    setInverterVoltages(L_INJECT_V*s,0);
    readCurrents(Ia,Ib);if(trip(Ia,Ib,"L"))return -1;
    sIs+=Ia*s;sIc+=Ia*c;
    ang+=da;if(ang>_2PI)ang-=_2PI;
  }
  setInverterVoltages(0,0);
  for(uint32_t i=0;i<40;i++) waitTick();
  float Ir=(float)(2.0*sIs/N),Ii=(float)(2.0*sIc/N);
  float Ipk=sqrtf(Ir*Ir+Ii*Ii);
  float Z=(Ipk>1e-6f)?(L_INJECT_V*(2.0f/3.0f)/Ipk):0;
  float phi=atan2f(-Ii,Ir)*180.0f/_PI;
  Serial.print("  [");Serial.print(tag);Serial.print("] Ipk=");
  Serial.print(Ipk*1000,2);Serial.print("mA  |Z|=");
  Serial.print(Z*1000,2);Serial.print("mOhm  phi=");
  Serial.print(phi,1);Serial.println("deg");
  return Z;
}

float measure_L1(float R_meas) {
  Serial.println("\n--- Phase 2: Inductance ---");
  float Z_lo=measure_L_at_freq(L_FREQ_LO,"800Hz"); if(Z_lo<0)return -1;
  float Z_hi=measure_L_at_freq(L_FREQ_HI,"9000Hz"); if(Z_hi<0)return -1;
  float wlo=_2PI*L_FREQ_LO, whi=_2PI*L_FREQ_HI;
  float L_lo=sqrtf(fmaxf(0,Z_lo*Z_lo-R_meas*R_meas))/wlo;
  float L_hi=sqrtf(fmaxf(0,Z_hi*Z_hi-R_meas*R_meas))/whi;
  Serial.print("  L@200Hz = ");Serial.print(L_lo*1e6,2);Serial.println(" uH");
  Serial.print("  L@800Hz = ");Serial.print(L_hi*1e6,2);Serial.println(" uH  <- used for FOC");
  // if(L_lo > 1.5f*L_hi)
  //   Serial.println("  (spread >1.5x: saturation confirmed, 800Hz value is more reliable)");
  return L_hi;
}

float measure_L(float R_meas) {
  Serial.println("\n--- Phase 2: Inductance Sweep ---");
  
  const float freqs[] = {10, 20, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
  const int num_freqs = sizeof(freqs) / sizeof(freqs[0]);
  
  float L_values[num_freqs];
  float Z_values[num_freqs];
  bool all_ok = true;
  
  for (int i = 0; i < num_freqs; i++) {
    float f = freqs[i];
    Serial.print("  Measuring at ");
    Serial.print(f);
    Serial.println(" Hz ...");
    
    // Create a C-string tag for the frequency
    char tag[20];
    snprintf(tag, sizeof(tag), "%.0fHz", f);
    
    float Z = measure_L_at_freq(f, tag);
    if (Z < 0) {
      Serial.print("    ERROR: measurement failed at ");
      Serial.println(f);
      all_ok = false;
      break;
    }
    Z_values[i] = Z;
    
    float w = 2.0f * 3.14159265f * f;
    float L = sqrtf(fmaxf(0, Z*Z - R_meas*R_meas)) / w;
    L_values[i] = L;
    
    Serial.print("    Z = ");
    Serial.print(Z, 3);
    Serial.print(" ohm,  L = ");
    Serial.print(L * 1e6, 2);
    Serial.println(" uH");
  }
  
  if (!all_ok) return -1.0f;
  
  float L_hi = L_values[num_freqs - 1];
  float L_lo = L_values[0];
  
  Serial.println("\n--- Summary ---");
  Serial.print("  L @ ");
  Serial.print(freqs[0]);
  Serial.print(" Hz = ");
  Serial.print(L_lo * 1e6, 2);
  Serial.println(" uH");
  Serial.print("  L @ ");
  Serial.print(freqs[num_freqs - 1]);
  Serial.print(" Hz = ");
  Serial.print(L_hi * 1e6, 2);
  Serial.println(" uH  <- used for FOC");
  
  if (L_lo > 1.5f * L_hi) {
    Serial.println("  (spread >1.5x: saturation confirmed, high-freq value is more reliable)");
  }
  
  return L_hi;
}

// ================================================================
// PHASE 3 — LAMBDA
//
// V/f law: V_mag = V_BASE + lambda_KV * omega_elec
//
// Physics: back-EMF = lambda * omega_elec
// If V_PER_RADS = lambda, the incremental voltage exactly tracks back-EMF,
// keeping I_ss = V_BASE/R = constant throughout the ramp.
// No transient spikes. No break voltage needed.
//
// At omega=1500 rad/s: V_mag = 0.46 + 0.00062*1500 = 1.39V  (safe)
// I_ss_spinning  ≈ V_BASE/R = 5A
// I_ss_stalled   ≈ V_mag/R  = 15A  -> stall threshold = 10A
// ================================================================
float measure_Lambda(float R_meas, float L_meas) {
  Serial.println("\n--- Phase 3: Lambda (back-EMF) ---");
  float V_at_speed = LAMBDA_V_BASE + LAMBDA_V_PER_RADS * LAMBDA_TARGET_SPEED;
  Serial.print("  V/f: "); Serial.print(LAMBDA_V_BASE,4);
  Serial.print(" + "); Serial.print(LAMBDA_V_PER_RADS*1000,4);
  Serial.print(" mV/rad/s  ->  V at ");
  Serial.print(LAMBDA_TARGET_SPEED,0); Serial.print(" rad/s = ");
  Serial.print(V_at_speed,3); Serial.println("V");
  Serial.print("  Expected I_spin ~ "); Serial.print(LAMBDA_V_BASE/R_meas,2);
  Serial.print("A  |  Stall threshold: "); Serial.print(I_STALL_THRESHOLD,1); Serial.println("A");

  float Ia,Ib,prev_Ia=0,prev_Ib=0,E_a=0,E_b=0;
  float fg=LOOP_DT/(LAMBDA_LPF_TAU+LOOP_DT);
  float ol_speed=0,ol_angle=0;
  uint32_t ticks_at_speed=0;
  bool stall_checked=false;
  double sum_lam=0; uint32_t n=0;
  float lambda_live=0;

  // Alignment — gentle ramp to V_BASE over 1s
  Serial.println("  Aligning...");
  for(uint32_t i=0;i<20000;i++){
    waitTick();
    setInverterVoltages((float)i/20000.0f*LAMBDA_V_BASE,0);
    readCurrents(Ia,Ib);if(trip(Ia,Ib,"align"))return -1;
  }

  Serial.println("  Spinning (V/f ramp, I ~ constant)...");
  for(;;){
    waitTick();
    if(ol_speed<LAMBDA_TARGET_SPEED) ol_speed+=LAMBDA_ACCEL*LOOP_DT;
    else { ol_speed=LAMBDA_TARGET_SPEED; ticks_at_speed++; }

    ol_angle+=ol_speed*LOOP_DT;
    if(ol_angle>_2PI)ol_angle-=_2PI;

    float sn,cs;
    get_Fast_Sin_Cos(ol_angle,&sn,&cs);

    // The V/f law: current stays ~ I_TARGET throughout
    float v_mag=LAMBDA_V_BASE+LAMBDA_V_PER_RADS*ol_speed;
    setInverterVoltages(-v_mag*sn, v_mag*cs);

    readCurrents(Ia,Ib);
    if(trip(Ia,Ib,"spin"))return -1;

    // Stall check at 500ms after reaching speed
    if(!stall_checked && ticks_at_speed==LAMBDA_SETTLE_TICKS){
      float Im=sqrtf(Ia*Ia+Ib*Ib);
      Serial.print("  Stall check: |I|=");Serial.print(Im,2);Serial.print("A  -> ");
      if(Im>I_STALL_THRESHOLD){
        setInverterVoltages(0,0);
        Serial.println("STALLED.");
        Serial.println("  Try: increase LAMBDA_V_BASE slightly (e.g. 0.6f).");
        Serial.println("  Or:  reduce LAMBDA_ACCEL (e.g. 50.0f) for slower ramp.");
        return -1;
      }
      stall_checked=true;
      Serial.println("SPINNING OK");
    }

    // Back-EMF extraction after stall confirmed
    float dIa=(Ia-prev_Ia)/LOOP_DT, dIb=(Ib-prev_Ib)/LOOP_DT;
    prev_Ia=Ia; prev_Ib=Ib;

    if(stall_checked){
      float v_alpha=-v_mag*sn, v_beta=v_mag*cs;
      float Ea=v_alpha-R_meas*Ia-L_meas*dIa;
      float Eb=v_beta -R_meas*Ib-L_meas*dIb;
      E_a=(1-fg)*E_a+fg*Ea;
      E_b=(1-fg)*E_b+fg*Eb;
      lambda_live=sqrtf(E_a*E_a+E_b*E_b)/ol_speed;
      sum_lam+=lambda_live; n++;
      if(n>=LAMBDA_MEASURE_TICKS) break;
      if((n%10000)==9999){
        Serial.print("  omega=");Serial.print(ol_speed,0);
        Serial.print(" V=");Serial.print(v_mag,3);
        Serial.print("V  Ia=");Serial.print(Ia,2);
        Serial.print("A  lambda=");Serial.print(lambda_live*1000,4);
        Serial.println(" mWb");
      }
    }
  }

  setInverterVoltages(0,0);delay(300);
  float lam=(float)(sum_lam/n);
  float kv_lam=60.0f/(_2PI*2200.0f*1.73205f*7.0f);
  Serial.print("  Lambda (meas)  = ");Serial.print(lam*1000,5);Serial.println(" mWb");
  Serial.print("  Lambda (KV)    = ");Serial.print(kv_lam*1000,5);Serial.println(" mWb");
  Serial.print("  Ratio meas/KV  = ");Serial.print(lam/kv_lam,3);
  Serial.println("  (expect 0.85-1.15)");
  return lam;
}

// ================================================================
// SETUP
// ================================================================
void setup(){
  hardware_lockdown();
  Serial.begin(2000000);delay(1000);
  RCC->AHB1ENR|=RCC_AHB1ENR_CORDICEN;
  CORDIC->CSR=(6<<CORDIC_CSR_PRECISION_Pos)|CORDIC_CSR_NRES;
  setup_bare_metal_ADCs();configureTIM1();
  TIM1->CCR1=2125;TIM1->CCR2=2125;TIM1->CCR3=2125;delay(200);
  long sU=0,sV=0;
  for(int i=0;i<4000;i++){while(!(TIM1->SR&TIM_SR_UIF));TIM1->SR&=~TIM_SR_UIF;sU+=ADC1->JDR1;sV+=ADC2->JDR1;}
  offset_U=(float)sU/4000.0f;offset_V=(float)sV/4000.0f;
  GPIOA->MODER=(GPIOA->MODER&~0x3F0000)|0x2A0000;
  GPIOA->AFR[1]=(GPIOA->AFR[1]&~0xFFF)|0x666;
  GPIOB->MODER=(GPIOB->MODER&~0xFC000000)|0xA8000000;
  GPIOB->AFR[1]=(GPIOB->AFR[1]&~0xFFF00000)|0x46600000;
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;

  float kv_lam=60.0f/(_2PI*2200.0f*7.0f);
  float V_at_speed=LAMBDA_V_BASE+LAMBDA_V_PER_RADS*LAMBDA_TARGET_SPEED;
  Serial.println("\n================================================");
  Serial.println(" SELF-COMMISSIONING V7.0");
  Serial.println("================================================");
  Serial.print("  offset_U=");Serial.print(offset_U,1);Serial.print("  offset_V=");Serial.println(offset_V,1);
  Serial.print("  λ_KV = ");Serial.print(kv_lam*1000,4);Serial.println(" mWb");
  Serial.print("  Spin V: ");Serial.print(LAMBDA_V_BASE,3);Serial.print("V base + ");
  Serial.print(LAMBDA_V_PER_RADS*1e6,0);Serial.print(" uV/rad/s  ->  ");
  Serial.print(V_at_speed,3);Serial.print("V at ");Serial.print(LAMBDA_TARGET_SPEED,0);
  Serial.println(" rad/s  (I_ss ~ const throughout ramp)");
  Serial.println("\nSend any character to start.");
}

void loop(){
  if(!Serial.available())return;
  while(Serial.available())Serial.read();
  Serial.println("\n>>> Commissioning...");
  float R=measure_R();          if(R<0){Serial.println("ABORTED at R.");      return;}
  float L=measure_L(R);         if(L<0){Serial.println("ABORTED at L.");      return;}
  float Lam=measure_Lambda(R,L);if(Lam<0){Serial.println("ABORTED at Lambda.");return;}
  float kv_lam=60.0f/(_2PI*2200.0f*1.73205f*7.0f);
  Serial.println("\n================================================");
  Serial.println(" RESULT:");
  Serial.println("================================================");
  Serial.print("const float RS     = ");Serial.print(R,  6);Serial.println("f;");
  Serial.print("const float LS     = ");Serial.print(L,  9);Serial.println("f;");
  Serial.print("const float LAMBDA = ");Serial.print(Lam,9);Serial.println("f;");
  Serial.println();
  Serial.print("  KV cross-check:  ");Serial.print(kv_lam,9);Serial.println("f;");
  Serial.print("  Ratio = ");Serial.print(Lam/kv_lam,3);
  Serial.println("  (if outside 0.8-1.2, use KV value)");
  Serial.println("================================================");
  Serial.println("Send any character to run again.");
}
