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
int baseSpeed = 115;
const uint16_t LARGE_ERROR_THRESHOLD = 1500;
const int largeErrorSlowdown = 35;
const uint16_t SHARP_TURN_THRESHOLD = 900;
const int sharpTurnSlowSpeed = -50;
const int sharpTurnFastSpeed = 70;
const unsigned long SHARP_TURN_LOCK_MS = 600;

int lockedTurnDirection = 0; // -1 = left, 1 = right, 0 = no locked turn
unsigned long lockedTurnUntil = 0;

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
  int leftPwm = constrain(abs(leftSpeed), -255, 255);
  int rightPwm = constrain(abs(rightSpeed), -255, 255);

  // Left motor (B)
  digitalWrite(BIN1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(BIN2, leftSpeed >= 0 ? LOW : HIGH);
  analogWrite(PWMB, leftPwm);

  // Right motor (A)
  digitalWrite(AIN1, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(AIN2, rightSpeed >= 0 ? LOW : HIGH);
  analogWrite(PWMA, rightPwm);
}

void runLockedTurn(int error) {
  if (lockedTurnDirection == -1) {
    moveForward(sharpTurnSlowSpeed, sharpTurnFastSpeed);
    Serial.print("Locked LEFT | Error: "); Serial.println(error);
  } else {
    moveForward(sharpTurnFastSpeed, sharpTurnSlowSpeed);
    Serial.print("Locked RIGHT | Error: "); Serial.println(error);
  }

  lastError = error;
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
  bool leftSixOnLine = sensorValues[0] > SHARP_TURN_THRESHOLD &&
                       sensorValues[1] > SHARP_TURN_THRESHOLD &&
                       sensorValues[2] > SHARP_TURN_THRESHOLD &&
                       sensorValues[3] > SHARP_TURN_THRESHOLD &&
                       sensorValues[4] > SHARP_TURN_THRESHOLD &&
                       sensorValues[5] > SHARP_TURN_THRESHOLD &&
                       sensorValues[7] < SHARP_TURN_THRESHOLD;
  bool rightSixOnLine = sensorValues[2] > SHARP_TURN_THRESHOLD &&
                        sensorValues[3] > SHARP_TURN_THRESHOLD &&
                        sensorValues[4] > SHARP_TURN_THRESHOLD &&
                        sensorValues[5] > SHARP_TURN_THRESHOLD &&
                        sensorValues[6] > SHARP_TURN_THRESHOLD &&
                        sensorValues[7] > SHARP_TURN_THRESHOLD &&
                        sensorValues[0] < SHARP_TURN_THRESHOLD;

  if (lockedTurnDirection != 0) {
    if (millis() < lockedTurnUntil) {
      runLockedTurn(error);
      return;
    }

    lockedTurnDirection = 0;
  }

  // Sharp turn logic: outer 3 sensors detect a clear left/right turn.
  // If both sides are black, treat it like a wide line/intersection and use normal PD.
  if (leftSixOnLine && !rightThreeOnLine) {
    lockedTurnDirection = -1;
    lockedTurnUntil = millis() + SHARP_TURN_LOCK_MS;
  } else if (rightSixOnLine && !leftThreeOnLine) {
    lockedTurnDirection = 1;
    lockedTurnUntil = millis() + SHARP_TURN_LOCK_MS;
  }

  if (lockedTurnDirection != 0) {
    runLockedTurn(error);
    return;
  }

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

  // 4. Slightly lower speed during bigger errors for smoother turns.
  int activeBaseSpeed = baseSpeed;
  if (abs(error) > LARGE_ERROR_THRESHOLD) {
    activeBaseSpeed = baseSpeed - largeErrorSlowdown;
  }

  // 5. Apply to motors
  int leftSpeed  = constrain(activeBaseSpeed + correction, 0, 255);
  int rightSpeed = constrain(activeBaseSpeed - correction, 0, 255);

  moveForward(leftSpeed, rightSpeed);

  // 6. Save error for next loop
  lastError = error;

  // 7. Debug
  Serial.print("Error: "); Serial.print(error);
  Serial.print(" | dError: "); Serial.print(dError);
  Serial.print(" | base: "); Serial.print(activeBaseSpeed);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.println(rightSpeed);
}
