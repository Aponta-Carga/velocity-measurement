#include <Arduino.h>
#include "Wire.h"

#define E3F_1 27
#define E3F_2 33


long int sensor1_time = 0, sensor2_time = 0;
bool readyToCalculate = false;
double speed = 0;

bool was_E3F_1_high = false, was_E3F_2_high = false;


void read_dual_sensors() {

  // Serial.print("1: ");
  // Serial.print(measure1.RangeMilliMeter);
  // Serial.print(" 2: ");
  // Serial.println(measure2.RangeMilliMeter);

  if (digitalRead(E3F_1) && !was_E3F_1_high) {
    sensor1_time = micros();
    Serial.println("Sensor 1 detected an object");
    was_E3F_1_high = true;
  }
  
  if (digitalRead(E3F_2) && !was_E3F_2_high) {
    sensor2_time = micros();
    Serial.println("Sensor 2 detected an object");
    was_E3F_1_high = false;
    readyToCalculate = true;
  }

  if(readyToCalculate) {
    Serial.print("tempo1: ");
    Serial.print(sensor1_time);
    Serial.print(" tempo2: ");
    Serial.print(sensor2_time);
    Serial.print(" diferenca: ");
    Serial.println(sensor2_time - sensor1_time);
    Serial.print("Velocidade: ");
    speed = (200.0/(sensor2_time - sensor1_time)) * 1000.0;
    Serial.println(speed);
    if(!digitalRead(E3F_1)) {
      was_E3F_1_high = false;
    }
    if(!digitalRead(E3F_2)) {
      was_E3F_2_high = false;
      readyToCalculate = false;
    }
  }

}

void setup() {
  Serial.begin(115200);

  while (! Serial) { delay(1); }

  pinMode(E3F_1, INPUT);
  pinMode(E3F_2, INPUT);

  Serial.println(F("Sensor's pin's inited..."));
  
  
  Serial.println(F("Starting..."));
}

void loop() {
  read_dual_sensors();
  delay(10);
}