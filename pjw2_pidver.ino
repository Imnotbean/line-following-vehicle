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
int leftThreshold  = 200;
int rightThreshold = 200;

// ====== Encoder settings（只用左边） ======
const int ENC_L = A4;

volatile long pL = 0;
volatile unsigned long lastUsL = 0;
const unsigned long MIN_EDGE_US = 300;

const float WHEEL_DIAMETER_M = 0.065f;
const int   PULSES_PER_REV   = 20;
const float DIST_PER_PULSE_M = 3.1415926f * WHEEL_DIAMETER_M / PULSES_PER_REV;

unsigned long startTime = 0;

// ====== PID VARIABLES ======
float Kp = 1500;        
float Ki = 0.0;         
float Kd = 4.5;

float lastError = 0;
float integral = 0;

int rightbaseSpeed = 235;   
int leftbaseSpeed = 200;
int rightmaxSpeed = 255;    
int leftmaxSpeed = 215;

// ====== IR Calibration ======
int L_min = 120, L_max = 660;
int R_min = 100, R_max = 640;

// ====== FIRST RUN CONTROL ======
bool firstRunDone = false;       // 是否已经走完10cm+停2s
bool firstRunStop = false;       // 是否已触发10cm的停车
unsigned long stopStartTime = 0; // 停车的时间点

// ====== LCD Freeze ======
bool freezeDisplay = false;
float freezeDist = 0;            // 冻结时的距离
unsigned long freezeTime = 0;    // 冻结时的时间

// ====== Encoder ISR ======
void onLeft(){
  unsigned long t = micros();
  if (digitalRead(ENC_L) == HIGH && (t - lastUsL > MIN_EDGE_US)) {
    pL++;
    lastUsL = t;
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
  attachPCINT(digitalPinToPCINT(ENC_L), onLeft, CHANGE);
}

void loop() {

  // ====== 当前实时距离、时间 ======
  long cL = pL;
  float dNow = cL * DIST_PER_PULSE_M * 100.0; // cm
  unsigned long elapsedSec = (millis() - startTime) / 1000;

  // =======================================================
  // 上电后的 FIRST RUN：先走到 10cm，然后停2秒
  // =======================================================
  if (!firstRunDone) {

    if (!firstRunStop) {
      // 距离 < 10 cm → 持续直走
      if (dNow < 10.0) {
        setMotor(leftbaseSpeed, rightbaseSpeed);
      } else {
        // 到达 >=10cm → 停车 & 冻结LCD显示
        stopMotors();
        firstRunStop = true;
        stopStartTime = millis();

        freezeDisplay = true;
        freezeDist = dNow;
        freezeTime = elapsedSec;
      }
    } 
    else {
      // 已经停住 → 等待满2秒
      stopMotors();

      if (millis() - stopStartTime >= 2000) {
        // 2秒结束 → 恢复正常
        firstRunDone = true;
        freezeDisplay = false;
      }
    }
  }

  // =======================================================
  // LCD 显示（包含冻结模式）
  // =======================================================
  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 200) {
    lastLCD = millis();

    lcd.setCursor(0,0);
    lcd.print("Avg:");

    if (freezeDisplay) {
      lcd.print((int)freezeDist);
    } else {
      lcd.print((int)dNow);
    }
    lcd.print("cm   ");

    lcd.setCursor(0,1);

    if (freezeDisplay) {
      lcd.print("Time:");
      lcd.print(freezeTime);
      lcd.print("s ");
      lcd.setCursor(10,1);
      lcd.print("Wait2s");
    } else {
      lcd.print("Time:");
      lcd.print(elapsedSec);
      lcd.print("s    ");
    }
  }

  // FIRST RUN 阶段 → 不进入循线逻辑
  if (!firstRunDone) return;

  // =======================================================
  // ======== 以下全部都是你的原始循线/转弯逻辑 ========
  // =======================================================

  int vL = analogRead(IR_L);
  int vR = analogRead(IR_R);
  int vM = digitalRead(IR_M);

  if (vL < leftThreshold && vR < rightThreshold && vM == 1){
    stopMotors();
    while(1){};
  }
  else if (vR < rightThreshold && vL > leftThreshold && vM == 1){
    stopMotors(); 
    delay(10);

    vL = analogRead(IR_L);
    vR = analogRead(IR_R);
    vM = digitalRead(IR_M);

    if (vL > leftThreshold && vR < rightThreshold && vM == 1) {
      perform90TurnRight(215);
    } 
  }
  else if (vL < leftThreshold && vR > rightThreshold && vM == 1){
    stopMotors(); 
    delay(10);

    vL = analogRead(IR_L);
    vR = analogRead(IR_R);
    vM = digitalRead(IR_M);

    if (vL < leftThreshold && vR > rightThreshold && vM == 1) {
      perform90TurnLeft(255);
    }
  }
  else {
    float normL = (float)(vL - L_min) / (float)(L_max - L_min);
    float normR = (float)(vR - R_min) / (float)(R_max - R_min);

    normL = constrain(normL, 0, 1);
    normR = constrain(normR, 0, 1);

    float error = normL - normR;

    integral += error;
    float derivative = error - lastError;
    lastError = error;

    float correction = Kp * error + Ki * integral + Kd * derivative;

    int leftSpeed  = leftbaseSpeed - correction;
    int rightSpeed = rightbaseSpeed + correction;

    leftSpeed  = constrain(leftSpeed,  0, leftmaxSpeed);
    rightSpeed = constrain(rightSpeed, 0, rightmaxSpeed);

    setMotor(leftSpeed, rightSpeed);
  }
}

// ================== MOTOR FUNCTIONS ==================
void setMotor(int leftSpd, int rightSpd) {
  analogWrite(enA, leftSpd);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  analogWrite(enB, rightSpd);
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

// ================== TURN FUNCTIONS ==================
void perform90TurnLeft(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, 165);
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  
  digitalWrite(in4, HIGH);

  delay(250);

  while(digitalRead(IR_M) == 0) {}
}

void perform90TurnRight(int speedVal) {
  analogWrite(enA, 195);
  analogWrite(enB, speedVal);
  digitalWrite(in1, LOW);  
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW);

  delay(250);

  while(digitalRead(IR_M) == 0) {}
}