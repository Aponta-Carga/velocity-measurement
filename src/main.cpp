

#include "Wire.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>


#define E3F_1 27
#define E3F_2 33



ros::NodeHandle nh;

std_msgs::Float32 msg;
ros::Publisher pub("sensor_data", &msg);


long int sensor1_time = 0, sensor2_time = 0;
bool readyToCalculate = false;
double speed = 0;

bool was_E3F_1_high = false, was_E3F_2_high = false;


void read_dual_sensors() {

  // Serial.print("1: ");
  // Serial.print(measure1.RangeMilliMeter);
  // Serial.print(" 2: ");
  // Serial.println(measure2.RangeMilliMeter);

  if (!digitalRead(E3F_1) && !was_E3F_1_high) {
    sensor1_time = micros();
    //Serial.println("Sensor 1 detected an object");
    was_E3F_1_high = true;
  }
  
  if (!digitalRead(E3F_2) && !was_E3F_2_high) {
    sensor2_time = micros();
    //Serial.println("Sensor 2 detected an object");
    was_E3F_1_high = false;
    readyToCalculate = true;
  }

  if(readyToCalculate) {
    //Serial.print("tempo1: ");
    //Serial.print(sensor1_time);
    //Serial.print(" tempo2: ");
    //Serial.print(sensor2_time);
    //Serial.print(" diferenca: ");
    //Serial.println(sensor2_time - sensor1_time);
    //Serial.print("Velocidade: ");
    speed = (200.0/(sensor2_time - sensor1_time)) * 1000.0;
    msg.data = speed;
    pub.publish(&msg);
    nh.spinOnce();

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

  nh.initNode();
  nh.advertise(pub);

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
