#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
volatile bool objectDetected = false;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  if (!sensor.begin()) {
    Serial.println("Failed to initialize VL53L0X sensor!");
    while (1);
  }
  
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), detectObject, FALLING);
  
  Serial.println("VL53L0X Test");
}

void loop() {
  if (objectDetected) {
    Serial.println("Object detected!");
    objectDetected = false;
  }
  
  delay(100);
}

void detectObject() {
  objectDetected = true;
}
