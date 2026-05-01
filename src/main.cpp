#include <Arduino.h>
#include <QTRSensors.h>

// --- QTR Setup ---
QTRSensors qtr;
const uint8_t SENSOR_COUNT = 8;
uint16_t sensorValues[SENSOR_COUNT];

// --- Motor Pins (TB6612) ---
// Motor A (right)
int PWMA = 3;
int AIN1 = 5;
int AIN2 = 4;
// Motor B (left)
int PWMB = 9;
int BIN1 = 6;
int BIN2 = 7;
int STBY = 2;

// --- PID Variables ---
float Kp = 0.05;
int baseSpeed = 120;

void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7}, SENSOR_COUNT);

  Serial.println("Calibrating...");
  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(25);
  }
  Serial.println("Done! Starting in 2 seconds...");
  delay(2000);
}

// --- Motor Functions ---
void moveForward(int leftSpeed, int rightSpeed) {
  // Left motor (B) forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, leftSpeed);

  // Right motor (A) forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, rightSpeed);
}

void loop() {
  // 1. Read position and calculate error
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = (int)position - 3500;

  // 2. P correction
  int correction = Kp * error;

  // 3. Apply to motors (keep speeds between 0 and 255)
  int leftSpeed  = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);

  moveForward(leftSpeed, rightSpeed);

  // 4. Debug
  Serial.print("Error: "); Serial.print(error);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.println(rightSpeed);
}