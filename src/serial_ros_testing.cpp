// #include <Wire.h>
// #include <Adafruit_VL53L0X.h>
// #include <ros_com.h>

// #define LED_BUILTIN 2

// #define LOX1_ADDRESS 0x30
// #define LOX2_ADDRESS 0x31

// #define SHT_LOX1 27
// #define SHT_LOX2 33

// #define DETECTION_DISTANCE_MM 850

// Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// VL53L0X_RangingMeasurementData_t measure1;
// VL53L0X_RangingMeasurementData_t measure2;

// long int sensor1_time = 0, sensor2_time = 0;
// int dist1_ant = 3000, dist2_ant = 3000;
// bool readyToCalculate = false;
// float speed = 0;

// void setID() {

//   digitalWrite(SHT_LOX1, LOW);    
//   digitalWrite(SHT_LOX2, LOW);
//   delay(10);

//   digitalWrite(SHT_LOX1, HIGH);
//   digitalWrite(SHT_LOX2, HIGH);
//   delay(10);

//   digitalWrite(SHT_LOX1, HIGH);
//   digitalWrite(SHT_LOX2, LOW);

//   if(!lox1.begin(LOX1_ADDRESS)) {
//     while(1);
//   }
//   delay(10);

//   digitalWrite(SHT_LOX2, HIGH);
//   delay(10);

//   if(!lox2.begin(LOX2_ADDRESS)) {
//     while(1);
//   }

// }

// void read_dual_sensors() {
  
//   lox1.rangingTest(&measure1, false);
//   lox2.rangingTest(&measure2, false);

//   Serial.print("1: ");
//   Serial.print(measure1.RangeMilliMeter);
//   Serial.print(" 2: ");
//   Serial.println(measure2.RangeMilliMeter);

//   if (measure1.RangeMilliMeter <= DETECTION_DISTANCE_MM && dist1_ant >= DETECTION_DISTANCE_MM) {
//     sensor1_time = micros();
//     dist1_ant = measure1.RangeMilliMeter;
//     digitalWrite(LED_BUILTIN, HIGH);
//   }
  
//   if (measure2.RangeMilliMeter <= DETECTION_DISTANCE_MM && dist2_ant >= DETECTION_DISTANCE_MM) {
//     sensor2_time = micros();
//     dist2_ant = measure2.RangeMilliMeter;
//     readyToCalculate = true;
//     digitalWrite(LED_BUILTIN, LOW);
//   }

//   if(readyToCalculate) {
//     // Serial.print("tempo1: ");
//     // Serial.print(sensor1_time);
//     // Serial.print(" tempo2: ");
//     // Serial.print(sensor2_time);
//     // Serial.print(" diferenca: ");
//     // Serial.println(sensor2_time - sensor1_time);
//     // Serial.print("Velocidade: ");
//     speed = (200.0/(sensor2_time - sensor1_time)) * 1000.0;
//     ros_loop(speed);
//     delay(100);
//     if(measure1.RangeMilliMeter >= DETECTION_DISTANCE_MM) {
//       dist1_ant = 3000;
//     }
//     if(measure2.RangeMilliMeter >= DETECTION_DISTANCE_MM) {
//       dist2_ant = 3000;
//       readyToCalculate = false;
//     }
//   }

// }

// void setup() {
//   // Serial.begin(115200);
//   // while (! Serial) { delay(1); }
//   ros_init();

//   Serial.println("INICIOU!!");

//   pinMode(LED_BUILTIN, OUTPUT);

//   pinMode(SHT_LOX1, OUTPUT);
//   pinMode(SHT_LOX2, OUTPUT);

//   Serial.println(F("Shutdown pins inited..."));

//   digitalWrite(SHT_LOX1, LOW);
//   digitalWrite(SHT_LOX2, LOW);

//   Serial.println(F("Both in reset mode...(pins are low)"));
  
  
//   Serial.println(F("Starting..."));
//   setID();
// }

// void loop() {
//   read_dual_sensors();
//   // float speed_test = 89.568;
//   // ros_loop(speed_test);
// }