// =================================================================
// BARE-METAL SELF-COMMISSIONING KERNEL V8.0
// Target: WeAct STM32G431 | A2212 2200KV 7pp
// 
// CHANGES:
// 1. Phase V is DEAD. Fully migrated to Phase U and Phase W.
// 2. Clarke Transform inverted to correctly handle Phase W hardware swap.
// 3. CORDIC NRES bug permanently patched via CMSIS macros.
// 4. Theoretical Lambda calculation fixed with sqrt(3) line-to-neutral.
// 5. Static AC Inductance extraction (1000Hz) bypasses the Id=0 open-loop paradox.
// =================================================================

#include <Arduino.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define _SQRT3 1.73205080757f
#define DRIVER_OFF HIGH

const float POWER_SUPPLY           = 12.0f;
const float LOOP_DT                = 50e-6f;
const float NOMINAL_AMPS_PER_COUNT = (3.3f / 4095.0f) / (0.002f * 20.0f);

float offset_U = 2048.0f;
float offset_W = 2048.0f; // Renamed for sanity

// --- COMMISSIONING PARAMETERS ---
const float    R_TEST_VOLTAGE  = 0.5f;
const uint32_t R_AVERAGE_TICKS = 2000;

const float    L_INJECT_V      = 0.4f;
const float    L_FREQ_HI       = 1000.0f; // 1kHz avoids AD8418 noise floor
const uint32_t L_N_CYCLES      = 20;

const float    LAMBDA_V_BASE        = 0.46f;   
const float    LAMBDA_V_PER_RADS    = 0.000620f; 
const float    LAMBDA_TARGET_SPEED  = 1500.0f; 
const float    LAMBDA_ACCEL         = 100.0f;  
const float    LAMBDA_LPF_TAU       = 0.005f;
const uint32_t LAMBDA_SETTLE_TICKS  = 10000;   
const uint32_t LAMBDA_MEASURE_TICKS = 40000;   
const float    I_STALL_THRESHOLD    = 10.0f;
const float    CURRENT_LIMIT_A      = 40.0f;

// ==========================================
// HARDWARE INITIALIZATION
// ==========================================
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

// THE CORDIC FIX
inline void get_Fast_Sin_Cos(float angle_rad, float* out_sin, float* out_cos) {
  while(angle_rad > _PI) angle_rad -= _2PI;
  while(angle_rad < -_PI) angle_rad += _2PI;
  int32_t angle_q31 = (int32_t)((angle_rad / _PI) * 2147483648.0f);
  CORDIC->WDATA = angle_q31;
  int32_t res_cos = CORDIC->RDATA;
  int32_t res_sin = CORDIC->RDATA;
  *out_cos = (float)res_cos / 2147483648.0f;
  *out_sin = (float)res_sin / 2147483648.0f;
}

// THE CLARKE FIX (PHASE W DOMINANT)
inline void readCurrents(float &Ia, float &Ib) {
  float IU = (float)((int32_t)ADC1->JDR1 - (int)offset_U) * NOMINAL_AMPS_PER_COUNT;
  float IW = (float)((int32_t)ADC2->JDR1 - (int)offset_W) * NOMINAL_AMPS_PER_COUNT;
  Ia = IU; 
  Ib = -(IU + 2.0f * IW) * 0.577350269f; // Solves the Phase W inversion
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
  for(uint32_t i=0; i<4000; i++){waitTick(); readCurrents(Ia,Ib); if(trip(Ia,Ib,"R"))return -1;}
  
  double sum=0;
  for(uint32_t i=0; i<R_AVERAGE_TICKS; i++){
    waitTick(); readCurrents(Ia,Ib); if(trip(Ia,Ib,"R"))return -1;
    sum+=Ia;
  }
  setInverterVoltages(0,0);
  for(uint32_t i=0; i<200; i++) waitTick();
  
  float I_avg=(float)(sum/R_AVERAGE_TICKS);
  float R=(R_TEST_VOLTAGE*(2.0f/3.0f))/I_avg;
  Serial.print("  I_avg=");Serial.print(I_avg,4);Serial.print("A  R=");Serial.print(R*1000,3);Serial.println(" mOhm");
  return R;
}

// ================================================================
// PHASE 2 — INDUCTANCE (1000Hz AC Sweep)
// ================================================================
float measure_L(float R_meas) {
  Serial.println("\n--- Phase 2: Inductance (1000Hz AC) ---");
  uint32_t tpc = (uint32_t)(1.0f / (L_FREQ_HI * LOOP_DT) + 0.5f);
  float da = _2PI * L_FREQ_HI * LOOP_DT, ang = 0;
  float Ia, Ib;
  
  for(uint32_t t=0; t<5*tpc; t++){
    waitTick(); setInverterVoltages(L_INJECT_V*sinf(ang),0);
    readCurrents(Ia,Ib); if(trip(Ia,Ib,"L"))return -1;
    ang+=da; if(ang>_2PI)ang-=_2PI;
  }
  
  double sIs=0, sIc=0; uint32_t N=L_N_CYCLES*tpc;
  for(uint32_t t=0; t<N; t++){
    waitTick();
    float s=sinf(ang), c=cosf(ang);
    setInverterVoltages(L_INJECT_V*s,0);
    readCurrents(Ia,Ib); if(trip(Ia,Ib,"L"))return -1;
    sIs+=Ia*s; sIc+=Ia*c;
    ang+=da; if(ang>_2PI)ang-=_2PI;
  }
  setInverterVoltages(0,0);
  for(uint32_t i=0; i<40; i++) waitTick();
  
  float Ir=(float)(2.0*sIs/N), Ii=(float)(2.0*sIc/N);
  float Ipk=sqrtf(Ir*Ir+Ii*Ii);
  float Z=(Ipk>1e-6f)?(L_INJECT_V*(2.0f/3.0f)/Ipk):0;
  
  float w = _2PI * L_FREQ_HI;
  float L = sqrtf(fmaxf(0, Z*Z - R_meas*R_meas)) / w;
  
  Serial.print("  [1000Hz] Ipk="); Serial.print(Ipk*1000,2); Serial.print("mA  |Z|=");
  Serial.print(Z*1000,2); Serial.println("mOhm");
  Serial.print("  L = "); Serial.print(L*1e6, 2); Serial.println(" uH");
  
  return L;
}

// ================================================================
// PHASE 3 — LAMBDA (OPEN-LOOP INTEGRATION)
// ================================================================
float measure_Lambda(float R_meas, float L_meas) {
  Serial.println("\n--- Phase 3: Lambda (Flux Linkage) ---");
  float V_at_speed = LAMBDA_V_BASE + LAMBDA_V_PER_RADS * LAMBDA_TARGET_SPEED;
  
  float Ia,Ib,prev_Ia=0,prev_Ib=0,E_a=0,E_b=0;
  float fg=LOOP_DT/(LAMBDA_LPF_TAU+LOOP_DT);
  float ol_speed=0,ol_angle=0;
  uint32_t ticks_at_speed=0;
  bool stall_checked=false;
  double sum_lam=0; uint32_t n=0;
  float lambda_live=0;

  Serial.println("  Aligning...");
  for(uint32_t i=0;i<20000;i++){
    waitTick();
    setInverterVoltages((float)i/20000.0f*LAMBDA_V_BASE,0);
    readCurrents(Ia,Ib);if(trip(Ia,Ib,"align"))return -1;
  }

  Serial.println("  Spinning...");
  for(;;){
    waitTick();
    if(ol_speed<LAMBDA_TARGET_SPEED) ol_speed+=LAMBDA_ACCEL*LOOP_DT;
    else { ol_speed=LAMBDA_TARGET_SPEED; ticks_at_speed++; }

    ol_angle+=ol_speed*LOOP_DT;
    if(ol_angle>_2PI)ol_angle-=_2PI;

    float sn,cs;
    get_Fast_Sin_Cos(ol_angle,&sn,&cs);

    float v_mag=LAMBDA_V_BASE+LAMBDA_V_PER_RADS*ol_speed;
    setInverterVoltages(-v_mag*sn, v_mag*cs);
    readCurrents(Ia,Ib);
    if(trip(Ia,Ib,"spin"))return -1;

    if(!stall_checked && ticks_at_speed==LAMBDA_SETTLE_TICKS){
      float Im=sqrtf(Ia*Ia+Ib*Ib);
      if(Im>I_STALL_THRESHOLD){
        setInverterVoltages(0,0);
        Serial.println("STALLED."); return -1;
      }
      stall_checked=true;
      Serial.println("  SPINNING OK");
    }

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
    }
  }

  setInverterVoltages(0,0);delay(300);
  float lam=(float)(sum_lam/n);
  
  // THE SQRT(3) FIX FOR TRUE THEORETICAL LAMBDA
  float kv_lam = 60.0f / (_SQRT3 * _2PI * 2200.0f * 7.0f);
  
  Serial.print("  Lambda (meas)  = ");Serial.print(lam*1000,5);Serial.println(" mWb");
  Serial.print("  Lambda (KV)    = ");Serial.print(kv_lam*1000,5);Serial.println(" mWb");
  return lam;
}

// ================================================================
// SETUP & MAIN
// ================================================================
void setup(){
  hardware_lockdown();
  Serial.begin(2000000);delay(1000);
  
  RCC->AHB1ENR|=RCC_AHB1ENR_CORDICEN;
  CORDIC->CSR=(6<<CORDIC_CSR_PRECISION_Pos)|CORDIC_CSR_NRES; // CORDIC FIXED
  
  setup_bare_metal_ADCs();configureTIM1();
  TIM1->CCR1=2125;TIM1->CCR2=2125;TIM1->CCR3=2125;delay(200);
  
  // PHASE W CALIBRATION FIX
  long sU=0,sW=0;
  for(int i=0;i<4000;i++){
    while(!(TIM1->SR&TIM_SR_UIF));TIM1->SR&=~TIM_SR_UIF;
    sU+=ADC1->JDR1; sW+=ADC2->JDR1;
  }
  offset_U=(float)sU/4000.0f; offset_W=(float)sW/4000.0f;
  
  GPIOA->MODER=(GPIOA->MODER&~0x3F0000)|0x2A0000;
  GPIOA->AFR[1]=(GPIOA->AFR[1]&~0xFFF)|0x666;
  GPIOB->MODER=(GPIOB->MODER&~0xFC000000)|0xA8000000;
  GPIOB->AFR[1]=(GPIOB->AFR[1]&~0xFFF00000)|0x46600000;

  Serial.println("\n================================================");
  Serial.println(" MASTER SELF-COMMISSIONING V8.0");
  Serial.println("================================================");
  Serial.print("  offset_U = ");Serial.print(offset_U,1);
  Serial.print("  offset_W = ");Serial.println(offset_W,1);
  Serial.println("\nSend any character to start extraction.");
}

void loop(){
  if(!Serial.available())return;
  while(Serial.available())Serial.read();
  
  Serial.println("\n>>> Commissioning...");
  float R = measure_R();          if(R<0){Serial.println("ABORTED at R.");      return;}
  float L = measure_L(R);         if(L<0){Serial.println("ABORTED at L.");      return;}
  float Lam = measure_Lambda(R,L);if(Lam<0){Serial.println("ABORTED at Lambda.");return;}
  
  float kv_lam = 60.0f / (_SQRT3 * _2PI * 2200.0f * 7.0f);
  
  Serial.println("\n================================================");
  Serial.println(" FINAL FLUX OBSERVER PARAMETERS:");
  Serial.println("================================================");
  Serial.print("const float RS     = ");Serial.print(R,  6);Serial.println("f;");
  Serial.print("const float LS     = ");Serial.print(L,  9);Serial.println("f;");
  Serial.print("const float LAMBDA = ");Serial.print(Lam,9);Serial.println("f;");
  Serial.println();
  Serial.print("  Theoretical KV   = ");Serial.print(kv_lam,9);Serial.println("f;");
  Serial.println("================================================");
  Serial.println("Send any character to run again.");
}