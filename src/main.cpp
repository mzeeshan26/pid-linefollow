#include <Arduino.h>
#include <QTRSensors.h>

// Arduino Uno has 6 analog inputs: A0..A5.
#define NUM_SENSORS 6

QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];
const uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5};

void setup() {
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);

  // Calibration: move sensor over line and white surface
  Serial.println("Calibrating... move sensor over surface");
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }
  Serial.println("Done");
}

void loop() {
  // Returns 0 (white) to 2500 (black) per sensor
  uint16_t position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  // position: 0 = far left, (NUM_SENSORS-1)*1000 = far right
  Serial.print("pos: ");
  Serial.println(position);

  delay(100);
}
