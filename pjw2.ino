#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>

LiquidCrystal lcd(8,9,4,5,6,7);

// ====== IR ======
const int IR_L = A1;
const int IR_R = A2;
const int IR_M = A5;

// ====== Motors ======
int enA = 3;        
int in1 = 1;
int in2 = 2;

int enB = 11;         
int in3 = 12;
int in4 = 13;

// ====== IR threshold ======
int leftThreshold  = 400;
int rightThreshold = 400;

// ====== Encoder settings ======
const int ENC_L = A4;
const int ENC_R = A3;

volatile long pL = 0, pR = 0;
volatile unsigned long lastUsL = 0, lastUsR = 0;
const unsigned long MIN_EDGE_US = 300;

const float WHEEL_DIAMETER_M = 0.065f;
const int   PULSES_PER_REV   = 20;
const float DIST_PER_PULSE_M = 3.1415926f * WHEEL_DIAMETER_M / PULSES_PER_REV;

// ====== Timer ======
unsigned long startTime = 0;

// ====== Encoder ISR ======
void onLeft(){
  unsigned long t = micros();
  if (digitalRead(ENC_L) == HIGH && (t - lastUsL > MIN_EDGE_US)) {
    pL++;
    lastUsL = t;
  }
}

void onRight(){
  unsigned long t = micros();
  if (digitalRead(ENC_R) == HIGH && (t - lastUsR > MIN_EDGE_US)) {
    pR++;
    lastUsR = t;
  }
}

void setup() {
  lcd.begin(16, 2);
  lcd.print("Line + Dist+Time");
  delay(1000);
  lcd.clear();

  startTime = millis();

  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);
  pinMode(IR_M, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(ENC_L), onLeft, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_R), onRight, CHANGE);
}

void loop() {

  // ============ 显示平均距离 + 时间 =============
  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 200) {
    lastLCD = millis();

    long cL = pL;
    long cR = pR;

    // 平均距离（米 → cm）
    float dAvg = ((cL + cR) / 2.0) * DIST_PER_PULSE_M * 100.0;

    // 时间（秒）
    unsigned long elapsedSec = (millis() - startTime) / 1000;

    lcd.setCursor(0,0);
    lcd.print("Avg:");
    lcd.print((int)dAvg);
    lcd.print("cm   ");

    lcd.setCursor(0,1);
    lcd.print("Time:");
    lcd.print(elapsedSec);
    lcd.print("s    ");
  }


  // ============ 以下是你的循迹逻辑，不动 ============
  int vL = analogRead(IR_L);
  int vR = analogRead(IR_R);
  int vM = digitalRead(IR_M);

  if (vL > leftThreshold && vR > rightThreshold && vM == 1) {
    moveForward(100);
  }
  else if (vL > leftThreshold && vR < rightThreshold && vM ==0 ) {
    turnLeft(150);  
  }
  else if (vR > rightThreshold && vL < leftThreshold && vM == 0) {
    turnRight(150);   
  }
  else if (vR < rightThreshold && vL < leftThreshold && vM == 1){
    stopMotors();
    while(1){};
  }
  else if (vR > rightThreshold && vL < leftThreshold && vM==1){
      findleftline(150);
  }
  else if (vR < rightThreshold && vL > leftThreshold && vM==1){
      findrightline(150);
  }
  else {
      
  }
  delay(50);
}


// =======================
// Motor functions（原封不动）
// =======================
void moveForward(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnLeft(int speedVal) {
  analogWrite(enA, speedVal);          
  analogWrite(enB, speedVal);   
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnRight(int speedVal) {
  analogWrite(enA, speedVal);   
  analogWrite(enB, speedVal);          
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void findrightline(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(1200);
}


void findleftline(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(1000);
}
void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}