// --- Pin Mapping ---
const int pinU_IN = PA10;  // Connected to 6N137 -> IR2104 Pin 2
const int pinSD   = PB9;  // Connected to PC817 -> IR2104 Pin 3 (All chips)

void setup() {
  Serial.begin(115200);
  
  pinMode(pinU_IN, OUTPUT);
  pinMode(pinSD, OUTPUT);

  // Ensure other phases are "Safe" (Assuming High = Low-side ON)
  pinMode(PA8, OUTPUT); digitalWrite(PA8, HIGH);
  pinMode(PA10, OUTPUT); digitalWrite(PA10, HIGH);

  Serial.println("--- IR2104 Pinout Truth Table Test ---");
  Serial.println("Measuring Phase U (PA9=IN, PB9=SD)");
}

void loop() {

  // --- STATE 2: IN=0, SD=5 ---
  digitalWrite(pinU_IN, LOW);
  digitalWrite(pinSD, HIGH);
  printState("IN: 5V | SD: 5V");
  delay(3000);

  // // --- STATE 4: IN=5, SD=5 ---
  // digitalWrite(pinU_IN, HIGH);
  // digitalWrite(pinSD, HIGH);
  // printState("IN: 0V | SD: 5V");
  // delay(3000);

  // // --- STATE 1: IN=0, SD=0 ---
  // digitalWrite(pinU_IN, LOW);
  // digitalWrite(pinSD, LOW);
  // printState("IN: 5V | SD: 0V");
  // delay(3000);

  // --- STATE 3: IN=5, SD=0 ---
  digitalWrite(pinU_IN, HIGH);
  digitalWrite(pinSD, LOW);
  printState("IN: 0V | SD: 0V");
  delay(3000);

}

void printState(String label) {
  Serial.println("-------------------------");
  Serial.println("Testing " + label);
  Serial.println("Check IR2104 Pin 7 (HO) and Pin 5 (LO)");
}