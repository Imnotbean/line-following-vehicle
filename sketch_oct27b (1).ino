#include <LiquidCrystal.h>
LiquidCrystal lcd(8,9,4,5,6,7);

// === IR Sensors (Analog) ===
const int IR_L = A1;
const int IR_R = A2;

// === Motor driver pins ===
int enA = 3;        // Left motor speed (PWM)
int in1 = 1;
int in2 = 2;

int enB = 11;         // Right motor speed (PWM)
int in3 = 12;
int in4 = 13;

// === Other settings ===
const unsigned long SAMPLE_MS = 200; // LCD update rate
const bool USE_SERIAL = false;       // true = also print to Serial

// === Thresholds (you can tune these) ===
int leftThreshold  = 350;  // for left IR sensor (max ≈ 500)
int rightThreshold = 600;  // for right IR sensor (max ≈ 750)

void setup() {
  lcd.begin(16, 2);
  lcd.print("IR Line Follower");
  delay(1000);
  lcd.clear();

  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);

  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  if (USE_SERIAL) {
    Serial.begin(9600);
    Serial.println("=== IR Line Follower ===");
  }
}

void loop() {
  // 1️⃣ Read sensor values
  int vL = analogRead(IR_L);
  int vR = analogRead(IR_R);

  // 2️⃣ Show on LCD (updated every SAMPLE_MS)
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last > SAMPLE_MS) {
    last = now;
    lcd.setCursor(0, 0);
    lcd.print("L:"); lcd.print(vL); lcd.print("    ");
    lcd.setCursor(0, 1);
    lcd.print("R:"); lcd.print(vR); lcd.print("    ");
  }

  if (USE_SERIAL) {
    Serial.print("L: "); Serial.print(vL);
    Serial.print(" | R: "); Serial.println(vR);
  }

  // 3️⃣ Basic line-following logic
  if (vL > leftThreshold && vR > rightThreshold) {
    // Both on line → forward
    moveForward(150);
  }
  else if (vL > leftThreshold && vR < rightThreshold) {
    // Line under left sensor → turn left
    turnLeft(120);
  }
  else if (vR > rightThreshold && vL < leftThreshold) {
    // Line under right sensor → turn right
    turnRight(120);
  }
  else if (vR < rightThreshold && vL < leftThreshold){
    // Lost line → stop
    stopMotors();
  }
  else {
    findline(150);
  }
}

// === Motor movement functions ===
void moveForward(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnLeft(int speedVal) {
  analogWrite(enA, speedVal);          // stop left
  analogWrite(enB, speedVal);   // move right
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnRight(int speedVal) {
  analogWrite(enA, speedVal);   // move left
  analogWrite(enB, speedVal);          // stop right
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void findline(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


