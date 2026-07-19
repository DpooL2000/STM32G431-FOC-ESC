#include <math.h>

const int HEATER = 9;
const int THERM  = A0;
const float TARGET_TEMP = 60.0; // Fast-heat to 60C

void setup() {
  pinMode(HEATER, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // 1. Read Thermistor
  int raw = analogRead(THERM);
  
  // 2. Simplified Temperature Calculation (Beta = 3950)
  // Assumes 10k fixed resistor connected to 5V, Thermistor to GND
  float R = 10000.0 / (1023.0 / (float)raw - 1.0);
  float temp = 1.0 / (log(R / 10000.0) / 3950.0 + 1.0 / 298.15) - 273.15;

  // 3. Bang-Bang Control Logic
  if (temp < TARGET_TEMP) {
    digitalWrite(HEATER, HIGH); // 100% Power for fastest heating
  } else {
    digitalWrite(HEATER,HIGH);
  }

  // 4. Monitor
  Serial.print("Temp: "); Serial.print(temp); 
  Serial.println(temp < TARGET_TEMP ? " C | HEATING..." : " C | TARGET REACHED");
  
  delay(200); 
}