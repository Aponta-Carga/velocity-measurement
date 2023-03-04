#pragma once

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define VL_ADDRES01 0X32
#define VL_ADDRES02 0X33

#define VL_PIN01 14
#define VL_PIN02 27

double enterTime;
double leaveTime;
double velocityMS;
double velocityKmH;

Adafruit_VL53L0X VL01 = Adafruit_VL53L0X();
Adafruit_VL53L0X VL02 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

void setup() {
  Serial.begin(115200);
  Wire.begin (21, 22);//sda, scl  

  pinMode(VL_PIN01, OUTPUT);
  pinMode(VL_PIN02, OUTPUT);

  digitalWrite(VL_PIN01, LOW); 
  digitalWrite(VL_PIN02, LOW);

  vTaskDelay(10);

  digitalWrite(VL_PIN01, HIGH); 
  digitalWrite(VL_PIN02, HIGH); 
  
  vTaskDelay(10);

  digitalWrite(VL_PIN01, HIGH);
  digitalWrite(VL_PIN02, LOW );
 
  vTaskDelay(10); 

  if(!VL01.begin(VL_ADDRES01)) {
    Serial.println("Failed to boot PRIMEIRO VL53L0X");
    while(1);
  }
  
  digitalWrite(VL_PIN02, HIGH);
  vTaskDelay(10); 
  
  if(!VL02.begin(VL_ADDRES02)) {
    Serial.println("Failed to boot SEGUNDO VL53L0X");
    while(1);
  }
  
}

void getVelocity() {
  VL01.rangingTest (& measure1, false);
  VL02.rangingTest (& measure2, false);
  
  if(measure1.RangeStatus != 4 && measure1.RangeMilliMeter <= 100){
    // When the first sensor measurement is smaller than 10cm, it records the enter time
    enterTime = micros();

    // While the second sensor measurement is greater than 10cm, it waits
    while(!(measure2.RangeStatus != 4 && measure2.RangeMilliMeter <= 100)) {
      VL02.rangingTest (& measure2, false);
    }
    // When the second sensor measurement is smaller than 10cm, it records the leave time
    leaveTime = micros();   
  }

  // All velolcity calculations are considering that the distance between the sensors is 200mm
  velocityMS = (200.0 / (leaveTime - enterTime)) * 1000.0;
  Serial.print("Velocidade(m/s): ");
  Serial.println(velocityMS,6);

  velocityKmH = (200.0 / (leaveTime - enterTime)) * 3600.0;
  Serial.print("Velocidade(km/h): ");
  Serial.println(velocityKmH,6);
}

void sendVelocity() {
  /*
    !!! TO DO !!!
    THIS FUNCTION SHOULD SEND THE VELOCITY CALCULATIONS VIA WIFI TO RASPBERRY 
    PI OR PUBLISH THE DATA DIRECTLY TO ROS.
  */
}

void loop(){
  
  getVelocity();
  sendVelocity();

}