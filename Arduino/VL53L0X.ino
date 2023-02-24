#pragma once

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define VL_ADDRES01 0X32
#define VL_ADDRES02 0X33

#define VL_PIN01 14
#define VL_PIN02 27

int tempoAntes;
int tempo;

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

void loop(){
  
  VL01.rangingTest (& measure1, false);
  VL02.rangingTest (& measure2, false);
  
  if(measure1.RangeStatus != 4){
    Serial.print("\nDistancia VL1 (mm): ");
    Serial.print(measure1.RangeMilliMeter);
    if(measure1.RangeMilliMeter <= 100) {
      tempoAntes = millis();
    } 
  }
  else {
    Serial.print("\n out of range ");
  }
  
  if(measure2.RangeStatus != 4){
    Serial.print(" | Distancia VL2 (mm): ");
    Serial.print(measure2.RangeMilliMeter);
    if(measure2.RangeMilliMeter <= 100) {
      tempo = millis();
    } 
  }else{
    Serial.print(" | out of range ");
  }

  Serial.println("");
  Serial.println(tempoAntes);
  Serial.println(tempo);
  Serial.print("Tempo de passagem: ");
  Serial.println(tempo - tempoAntes);
  Serial.print("Velocidade: ");
  Serial.println(100.0 / (tempo - tempoAntes));

}