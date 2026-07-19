// =================================================================
// FULL DIAGNOSTIC HFI TELEMETRY V8.1
// Target: WeAct STM32G431
// FIX: ADC Clock lowered to AHB/4 (42.5MHz - 47.5MHz) for stability
// =================================================================
#include <Arduino.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f
#define DRIVER_OFF HIGH

const float POWER_SUPPLY = 12.0f;
const float LOOP_DT      = 50e-6f; // 20kHz
const float RS           = 0.088560f; // Your measured Resistance

const float SHUNT_RESISTOR = 0.002f;
const float AMP_GAIN       = 20.0f;
const float NOMINAL_AMPS_PER_COUNT = (3.3f / 4095.0f) / (SHUNT_RESISTOR * AMP_GAIN);

float offset_U = 2049.5f;
float offset_W = 2043.0f;

// --- FULL DIAGNOSTIC TELEMETRY PAYLOAD (30 Bytes) ---
struct Telemetry {
  uint16_t sync = 0xABCD; // Magic word
  float iu;
  float iv;
  float iw;
  float i_alpha;
  float i_beta;
  float l_live;
  float l_min;
} __attribute__((packed));

Telemetry t_data;

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
  TIM1->CCER=0xFFF;
  TIM1->BDTR=0x80A0; // OLD (160 ticks = ~1128ns depending on DTG math)
  //TIM1->BDTR=0x8064; // NEW (100 ticks = 588ns)
  TIM1->CR2=0xF3F;TIM1->CR2&=~TIM_CR2_MMS_Msk;
  TIM1->CR2|=(0x7<<TIM_CR2_MMS_Pos);
  TIM1->EGR|=1;TIM1->CR1|=1;
}

void setup_bare_metal_ADCs() {
  RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR|=RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN|RCC_AHB2ENR_DAC3EN|RCC_AHB2ENR_ADC12EN;
  
  // PA1=analog  PA2=AF  PA4=analog  PA6=analog  PA7=analog
  GPIOA->MODER |= (3<<2) | (3<<4) | (3<<8) | (3<<12) | (3<<14);
  GPIOB->MODER|=(3<<0);
  GPIOA->PUPDR&=~((3<<4)|(3<<8)|(3<<12));
  
  // OPAMP1 / DAC3 setup for 1.65V Reference output on PA2
  DAC3->CR&=~DAC_CR_EN1;DAC3->MCR&=~DAC_MCR_MODE1_Msk;
  DAC3->MCR|=(3<<DAC_MCR_MODE1_Pos);
  DAC3->CR|=DAC_CR_EN1;DAC3->DHR12R1=2048;
  OPAMP1->CSR=(3<<5)|(3<<2);__asm__("nop");__asm__("nop");OPAMP1->CSR|=(1<<0);
  
  // --- THE ADC CLOCK FIX ---
  ADC12_COMMON->CCR &= ~(3 << 16); 
  ADC12_COMMON->CCR |= (3 << 16); // CKMODE = 11 (AHB/4). ADC Clock = ~47.5MHz!
  
  // 1. WAKE UP THE ADC CORE FIRST
  ADC1->CR&=~ADC_CR_DEEPPWD;ADC2->CR&=~ADC_CR_DEEPPWD;delay(1);
  ADC1->CR|=ADC_CR_ADVREGEN;ADC2->CR|=ADC_CR_ADVREGEN;delay(1);
  
  // 2. NOW CLEAR THE HARDWARE OFFSETS (Registers are now writable)
  ADC1->OFR1 = 0;
  ADC1->OFR2 = 0;
  ADC2->OFR1 = 0;
  
  // 3. CALIBRATE AND ENABLE
  ADC1->CR&=~ADC_CR_ADCALDIF;ADC1->CR|=ADC_CR_ADCAL;while(ADC1->CR&ADC_CR_ADCAL);
  ADC2->CR&=~ADC_CR_ADCALDIF;ADC2->CR|=ADC_CR_ADCAL;while(ADC2->CR&ADC_CR_ADCAL);delay(1);
  ADC1->ISR|=ADC_ISR_ADRDY;ADC1->CR|=ADC_CR_ADEN;while(!(ADC1->ISR&ADC_ISR_ADRDY));
  ADC2->ISR|=ADC_ISR_ADRDY;ADC2->CR|=ADC_CR_ADEN;while(!(ADC2->ISR&ADC_ISR_ADRDY));
  
  ADC1->SMPR1|=(6<<6);ADC1->SMPR2|=(6<<15);ADC2->SMPR1|=(6<<12);
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

void calibrate_AD8418_offsets() {
  Serial.println("Calibrating offsets... (entered function)");

  // --- DISABLE ALL PWM OUTPUTS (MOE = 0) ---
  uint32_t bdtr_backup = TIM1->BDTR;
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  delay(200); // let currents decay

  long sum_U = 0, sum_W = 0;
  const int N = 2000;
  int timeout = 0;

  // Wait for first update event with timeout
  while (!(TIM1->SR & TIM_SR_UIF)) {
    delayMicroseconds(10);
    if (++timeout > 100000) { // ~1 second timeout
      Serial.println("ERROR: TIM1 update event timeout!");
      return;
    }
  }
  TIM1->SR &= ~TIM_SR_UIF;

  // Now collect N samples
  for (int i = 0; i < N; i++) {
    while (!(TIM1->SR & TIM_SR_UIF)) {
      // small timeout to avoid infinite hang
      delayMicroseconds(5);
    }
    TIM1->SR &= ~TIM_SR_UIF;
    sum_U += ADC1->JDR1;
    sum_W += ADC2->JDR1;
  }

  offset_U = (float)sum_U / (float)N;
  offset_W = (float)sum_W / (float)N;

  // --- RE-ENABLE PWM OUTPUTS ---
  TIM1->BDTR = bdtr_backup;

  Serial.print("Offset U: "); Serial.println(offset_U, 2);
  Serial.print("Offset W: "); Serial.println(offset_W, 2);
  Serial.println("Calibration done (zero-current).");
}

void setup(){
  hardware_lockdown();
  Serial.begin(2000000); 
  setup_bare_metal_ADCs();
  configureTIM1();
  
  // 1. SET THE BASELINE TO 50% (0V Line-to-Line)
  TIM1->CCR1 = 2125; TIM1->CCR2 = 2125; TIM1->CCR3 = 2125;
  
  // 2. CONNECT PINS TO THE TIMER NOW (Start hammering the FETs)
  GPIOA->MODER=(GPIOA->MODER&~0x3F0000)|0x2A0000;
  GPIOA->AFR[1]=(GPIOA->AFR[1]&~0xFFF)|0x666;
  GPIOB->MODER=(GPIOB->MODER&~0xFC000000)|0xA8000000;
  GPIOB->AFR[1]=(GPIOB->AFR[1]&~0xFFF00000)|0x46600000;
  
  // Give the bootstrap capacitors 200ms to charge and stabilize
  delay(200); 
  
  // 3. CALIBRATE UNDER LIVE FIRE
  calibrate_AD8418_offsets();

  delay(1000);
}

void loop(){
  static float angle = 0;
  const float freq = 1000.0f; // 1kHz injection
  const float V_inj = 0.5f;   // 0.5V Amplitude
  static float I_max = -999.0f, I_min = 999.0f;
  static uint32_t cycle_count = 0;
  static float current_L = 0;
  
  static float min_L = 999.0f; 

  while (!(TIM1->SR & TIM_SR_UIF));
  TIM1->SR &= ~TIM_SR_UIF; 

  // Read U and W, reconstruct V via Kirchhoff
  float IU = (float)((int32_t)ADC1->JDR1 - (int)offset_U) * NOMINAL_AMPS_PER_COUNT;
  float IW = (float)((int32_t)ADC2->JDR1 - (int)offset_W) * NOMINAL_AMPS_PER_COUNT;
  float IV = -(IU + IW); // Kirchhoff's Current Law
  
  // Clarke Transform
  float I_alpha = IU; 
  float I_beta  = -(IU + 2.0f * IW) * 0.577350269f;

  // Track Peak-to-Peak AC current on Alpha axis
  if (I_alpha > I_max) I_max = I_alpha;
  if (I_alpha < I_min) I_min = I_alpha;

  // 1kHz Sine Wave Math
  angle += _2PI * freq * LOOP_DT;
  if (angle > _2PI) angle -= _2PI;
  float s = sinf(angle);
  
  // Inject Voltage
  setInverterVoltages(V_inj * s, 0.0f);

  // Every 20 loop ticks (1 period of 1000Hz)
  cycle_count++;
  if (cycle_count >= 20) {
    float I_pk = (I_max - I_min) / 2.0f;
    
    float Z = (I_pk > 0.01f) ? (V_inj * (2.0f/3.0f)) / I_pk : 0.0f;
    float w = _2PI * freq;
    
    if (Z*Z > RS*RS) {
       current_L = sqrtf(Z*Z - RS*RS) / w;
       
       if (current_L < min_L && current_L > 0.000005f) {
           min_L = current_L;
       }
    }

    I_max = -999.0f; I_min = 999.0f;
    cycle_count = 0;
  }
  static float iu_dc = 0, iw_dc = 0;
  const float alpha = 0.0005f;  // time constant ~2 seconds
  iu_dc = iu_dc * (1-alpha) + alpha * IU;
  iw_dc = iw_dc * (1-alpha) + alpha * IW;
  float IU_ac = IU - iu_dc;
  float IW_ac = IW - iw_dc;
  float IV_ac = -(IU_ac + IW_ac);
// Then send IU_ac, IW_ac (and recalc IV_ac) to the GUI

  // Downsample Telemetry to 1kHz
  if (cycle_count == 0) {
    t_data.iu = IU_ac;
    t_data.iv = IV_ac;
    t_data.iw = IW_ac;
    t_data.i_alpha = I_alpha;
    t_data.i_beta  = I_beta;
    t_data.l_live  = current_L * 1000000.0f; 
    t_data.l_min   = (min_L == 999.0f) ? 0.0f : (min_L * 1000000.0f); 
    
    Serial.write((uint8_t*)&t_data, sizeof(Telemetry));

  }
}