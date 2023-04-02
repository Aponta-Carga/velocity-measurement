#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#define SCL_SDIST 22
#define SDA_SDIST 21

#define SHT_1 33
#define SHT_2 25

VL53L0X sensor1;
VL53L0X sensor2;

int dist1;
int dist2;

void sensorsInit() {

  Wire.begin(21,22);

  pinMode(SHT_1, OUTPUT);
  pinMode(SHT_2, OUTPUT);

  digitalWrite(SHT_1, LOW);
  digitalWrite(SHT_2, LOW);

  pinMode(SHT_1, INPUT);
  sensor1.init(true);
  sensor1.setAddress((uint8_t)0x21);

  pinMode(SHT_2, INPUT);
  sensor2.init(true);
  sensor2.setAddress((uint8_t)0x22);

  sensor1.setTimeout(20);
  sensor2.setTimeout(20);

}

void distanceRead() {
  dist1 = sensor1.readRangeSingleMillimeters();
  dist2 = sensor2.readRangeSingleMillimeters();
}

void printDistances() {
  Serial.print("1: ");
  Serial.println(dist1);
  Serial.print("\t");
  Serial.print("2: ");
  Serial.println(dist2);
  Serial.print("\t");
}

void setup() {
  Serial.begin(115200);
  sensorsInit();
}

void loop() {
  distanceRead();
  printDistances();
}

// // #include <Wire.h>
// // #include <Adafruit_VL53L0X.h>
// // #include <ros_com.h>

// // /* Endereços */
// // #define LOX1_ADDRESS 0x30

// // /* Pinos de Shut */
// // #define SHT_LOX1 33

// // /* Range de decteção */
// // #define DETECTION_DISTANCE_MM 800

// // /* Objetos */
// // Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();

// // /* Distâncias */
// // VL53L0X_RangingMeasurementData_t firstSensorMeasurement;

// // void setID();
// // void readSensor();

// // void setup() {
// //   Serial.begin(115200);
// //   Serial.println("Inicio do programa - setup!!");
// //   ros_init();
// //   while (! Serial) { delay(1); }
// //   pinMode(SHT_LOX1, OUTPUT);
// //   Serial.println(F("Shutdown pins inited..."));
// //   digitalWrite(SHT_LOX1, LOW);
// //   Serial.println(F("Both in reset mode...(pins are low)"));
// //   Serial.println(F("Starting..."));
// //   setID();
// // }

// // void loop() {
// //   readSensor();
// //   delay(10);
// // }

// // void setID() {
// //   digitalWrite(SHT_LOX1, LOW);    
// //   delay(10);

// //   digitalWrite(SHT_LOX1, HIGH);
// //   delay(10);

// //   digitalWrite(SHT_LOX1, HIGH);

// //   if(!lox1.begin(LOX1_ADDRESS)) {
// //     Serial.println(F("Failed to boot first VL53L0X"));
// //     while(1);
// //   }
// //   delay(10);

// //   Serial.println("Configuracao dos IDs feita com sucesso!!");
// // }

// // void readSensor() {
// //   lox1.rangingTest(&firstSensorMeasurement, false);
// //   Serial.print("1: ");
// //   Serial.print(firstSensorMeasurement.RangeMilliMeter);
// // }
// // // ---------------------------------------------------------------------------------
// // // #define LOX1_ADDRESS 0x30
// // // #define LOX2_ADDRESS 0x31
// // // #define LOX3_ADDRESS 0x32

// // // #define SHT_LOX1 33
// // // #define SHT_LOX2 25
// // // #define SHT_LOX3 32

// // // #define DETECTION_DISTANCE_MM 800

// // // Adafruit_VL53L0X firstVL = Adafruit_VL53L0X();
// // // Adafruit_VL53L0X secondVL = Adafruit_VL53L0X();
// // // Adafruit_VL53L0X thirdVL = Adafruit_VL53L0X();

// // // VL53L0X_RangingMeasurementData_t firstSensorMeasurement;
// // // VL53L0X_RangingMeasurementData_t secondSensorMeasurement;
// // // VL53L0X_RangingMeasurementData_t thirdSensorMeasurement;

// // // long int sensor1_time = 0, sensor2_time = 0, sensor3_time = 0;
// // // int prev_dist1 = 3000, prev_dist2 = 3000, prev_dist3 = 3000;
// // // bool readyToCalculate = false;
// // // double speed = 0;

// // // void setID() {

// // //   digitalWrite(SHT_LOX1, LOW);    
// // //   digitalWrite(SHT_LOX2, LOW);
// // //   digitalWrite(SHT_LOX3, LOW);
// // //   delay(10);

// // //   digitalWrite(SHT_LOX1, HIGH);
// // //   digitalWrite(SHT_LOX2, HIGH);
// // //   digitalWrite(SHT_LOX3, HIGH);
// // //   delay(10);

// // //   digitalWrite(SHT_LOX1, HIGH);
// // //   digitalWrite(SHT_LOX2, LOW);
// // //   digitalWrite(SHT_LOX3, LOW);

// // //   if(!firstVL.begin(LOX1_ADDRESS)) {
// // //     while(1);
// // //   }
// // //   delay(10);

// // //   digitalWrite(SHT_LOX2, HIGH);
// // //   delay(10);

// // //   if(!secondVL.begin(LOX2_ADDRESS)) {
// // //     while(1);
// // //   }
// // //   delay(10);

// // //   digitalWrite(SHT_LOX3, HIGH);
// // //   delay(10);

// // //   if(!thirdVL.begin(LOX3_ADDRESS)) {
// // //     while(1);
// // //   }
// // //   delay(10);
// // //   Serial.println("Configuracao dos IDs feita com sucesso!!");
// // // }

// // // void read_dual_sensors() {
  
// // //   firstVL.rangingTest(&firstSensorMeasurement, false);
// // //   secondVL.rangingTest(&secondSensorMeasurement, false);
// // //   thirdVL.rangingTest(&thirdSensorMeasurement, false);

// // //   Serial.print("1: ");
// // //   // Serial.print(firstSensorMeasurement.RangeMilliMeter);
// // //   Serial.print(" 2: ");
// // //   // Serial.println(secondSensorMeasurement.RangeMilliMeter);
// // //   Serial.print(" 3: ");
// // //   // Serial.println(thirdSensorMeasurement.RangeMilliMeter);
// // //   if (firstSensorMeasurement.RangeMilliMeter <= DETECTION_DISTANCE_MM && prev_dist1 >= DETECTION_DISTANCE_MM) {
// // //     sensor1_time = micros();
// // //     prev_dist2 = firstSensorMeasurement.RangeMilliMeter;
// // //   }
  
// // //   if (secondSensorMeasurement.RangeMilliMeter <= DETECTION_DISTANCE_MM && prev_dist2 >= DETECTION_DISTANCE_MM) {
// // //     sensor2_time = micros();
// // //     prev_dist2 = secondSensorMeasurement.RangeMilliMeter;
// // //     readyToCalculate = true;
// // //   }

// // //   if(readyToCalculate) {
// // //     // Serial.print("tempo1: ");
// // //     // Serial.print(sensor1_time);
// // //     // Serial.print(" tempo2: ");
// // //     // Serial.print(sensor2_time);
// // //     // Serial.print(" diferenca: ");
// // //     // Serial.println(sensor2_time - sensor1_time);
// // //     // Serial.print("Velocidade: ");
// // //     speed = (200.0/(sensor2_time - sensor1_time)) * 1000.0;
// // //     // Serial.println(speed);
// // //     if(firstSensorMeasurement.RangeMilliMeter >= DETECTION_DISTANCE_MM) {
// // //       prev_dist1 = 3000;
// // //     }
// // //     if(secondSensorMeasurement.RangeMilliMeter >= DETECTION_DISTANCE_MM) {
// // //       prev_dist2 = 3000;
// // //       readyToCalculate = false;
// // //     }
// // //   }

// // // }

// // // void setup() {
// // //   Serial.begin(115200);
// // //   Serial.println("Iniciado o programa!!");
// // //   // ros_init();
// // //   while (! Serial) { delay(1); }
// // //   pinMode(SHT_LOX1, OUTPUT);
// // //   pinMode(SHT_LOX2, OUTPUT);
// // //   pinMode(SHT_LOX3, OUTPUT);
// // //   Serial.println(F("Shutdown pins inited..."));
// // //   digitalWrite(SHT_LOX1, LOW);
// // //   digitalWrite(SHT_LOX2, LOW);
// // //   digitalWrite(SHT_LOX3, LOW);
// // //   Serial.println(F("Both in reset mode...(pins are low)"));
// // //   Serial.println(F("Starting..."));
// // //   setID();
// // // }

// // // void loop() {
// // //   read_dual_sensors();
// // //   ros_loop(speed);
// // //   // delay(10);
// // // }