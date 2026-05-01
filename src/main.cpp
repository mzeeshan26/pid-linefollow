#include <Arduino.h>
#include <QTRSensors.h>

// --- QTR Setup ---
QTRSensors qtr;
const uint8_t SENSOR_COUNT = 8;
uint16_t sensorValues[SENSOR_COUNT];

// --- Motor Pins (TB6612) ---
int PWMA = 3;
int AIN1 = 5;
int AIN2 = 4;
int PWMB = 9;
int BIN1 = 6;
int BIN2 = 7;
int STBY = 2;

// --- PID Values ---
float Kp = 0.05;   // your working Kp value
float Kd = 0.11;    // start here, tune upward
int baseSpeed = 150;
const uint16_t SHARP_TURN_THRESHOLD = 900;
const int sharpTurnSlowSpeed = 0;
const int sharpTurnFastSpeed = 120;

// --- D needs last error ---
int lastError = 0;

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
  for (int i = 0; i < 100; i++) {
    qtr.calibrate();
    delay(25);
  }
  Serial.println("Done! Starting in 2 seconds...");
  delay(1000);
}

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
  bool leftThreeOnLine = sensorValues[0] > SHARP_TURN_THRESHOLD &&
                         sensorValues[1] > SHARP_TURN_THRESHOLD &&
                         sensorValues[2] > SHARP_TURN_THRESHOLD;
  bool rightThreeOnLine = sensorValues[5] > SHARP_TURN_THRESHOLD &&
                          sensorValues[6] > SHARP_TURN_THRESHOLD &&
                          sensorValues[7] > SHARP_TURN_THRESHOLD;

  // Sharp turn logic: outer 3 sensors detect a clear left/right turn.
  // If both sides are black, treat it like a wide line/intersection and use normal PD.
  if (leftThreeOnLine && !rightThreeOnLine) {
    moveForward(sharpTurnSlowSpeed, sharpTurnFastSpeed);
    lastError = error;

    Serial.print("Sharp LEFT | S0: "); Serial.print(sensorValues[0]);
    Serial.print(" S1: "); Serial.print(sensorValues[1]);
    Serial.print(" S2: "); Serial.print(sensorValues[2]);
    Serial.print(" | Error: "); Serial.println(error);
    return;
  }

  if (rightThreeOnLine && !leftThreeOnLine) {
    moveForward(sharpTurnFastSpeed, sharpTurnSlowSpeed);
    lastError = error;

    Serial.print("Sharp RIGHT | S5: "); Serial.print(sensorValues[5]);
    Serial.print(" S6: "); Serial.print(sensorValues[6]);
    Serial.print(" S7: "); Serial.print(sensorValues[7]);
    Serial.print(" | Error: "); Serial.println(error);
    return;
  }

  // 2. D — how fast is error changing
  int dError = error - lastError;

  // 3. PD correction
  int correction = (Kp * error) + (Kd * dError);

  // 4. Apply to motors
  int leftSpeed  = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);

  moveForward(leftSpeed, rightSpeed);

  // 5. Save error for next loop
  lastError = error;

  // 6. Debug
  Serial.print("Error: "); Serial.print(error);
  Serial.print(" | dError: "); Serial.print(dError);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.println(rightSpeed);
}
