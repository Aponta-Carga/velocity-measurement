// #include <Wire.h>
// #include <Adafruit_VL53L0X.h>

// // address we will assign if dual sensor is present
// #define LOX1_ADDRESS 0x30
// #define LOX2_ADDRESS 0x31

// // set the pins to shutdown
// #define SHT_LOX1 27
// #define SHT_LOX2 33

// #define DETECTION_DISTANCE_MM 500

// // objects for the vl53l0x
// Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// // this holds the measurement
// VL53L0X_RangingMeasurementData_t measure1;
// VL53L0X_RangingMeasurementData_t measure2;

// long int sensor1_time = 0, sensor2_time = 0;
// int dist1_ant = 1000, dist2_ant = 1000;
// bool readyToCalculate = false, readyToCalculate2 = false;
// double speed = 0;

// void setID() {
//   // all reset
//   digitalWrite(SHT_LOX1, LOW);    
//   digitalWrite(SHT_LOX2, LOW);
//   delay(10);
//   // all unreset
//   digitalWrite(SHT_LOX1, HIGH);
//   digitalWrite(SHT_LOX2, HIGH);
//   delay(10);

//   // activating LOX1 and resetting LOX2
//   digitalWrite(SHT_LOX1, HIGH);
//   digitalWrite(SHT_LOX2, LOW);

//   // initing LOX1
//   if(!lox1.begin(LOX1_ADDRESS)) {
//     Serial.println(F("Failed to boot first VL53L0X"));
//     while(1);
//   }
//   delay(10);

//   // activating LOX2
//   digitalWrite(SHT_LOX2, HIGH);
//   delay(10);

//   //initing LOX2
//   if(!lox2.begin(LOX2_ADDRESS)) {
//     Serial.println(F("Failed to boot second VL53L0X"));
//     while(1);
//   }

// }

// void read_dual_sensors() {
  
//   lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
//   lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

//   Serial.print("1: ");
//   Serial.print(measure1.RangeMilliMeter);
//   Serial.print(" 2: ");
//   Serial.println(measure2.RangeMilliMeter);

//   if (measure1.RangeMilliMeter <= DETECTION_DISTANCE_MM && dist1_ant >= DETECTION_DISTANCE_MM) {
//     sensor1_time = millis();
//     Serial.print("Sensor 1 detected an object at ");
//     Serial.print(sensor1_time);
//     Serial.println(" ms");
//     dist1_ant = measure1.RangeMilliMeter;
//     readyToCalculate = true;
//   }
  
//   if (measure2.RangeMilliMeter <= DETECTION_DISTANCE_MM && dist2_ant >= DETECTION_DISTANCE_MM) {
//     sensor2_time = millis();
//     Serial.print("Sensor 2 detected an object at ");
//     Serial.print(sensor2_time);
//     Serial.println(" ms");
//     dist2_ant = measure2.RangeMilliMeter;
//     readyToCalculate2 = true;
//   }

//   if(readyToCalculate2) {
//     Serial.println("PODEMOS CALCULAR A VELOCIDADE");
//     Serial.print("Velocidade: ");
//     speed = 200.0/(sensor2_time - sensor1_time);
//     Serial.println(speed);
//     if(measure1.RangeMilliMeter >= DETECTION_DISTANCE_MM) {
//       dist1_ant = 1000;
//       readyToCalculate = false;
//     }
//     if(measure2.RangeMilliMeter >= DETECTION_DISTANCE_MM) {
//       dist2_ant = 1000;
//       readyToCalculate2 = false;
//     }
//   }

//   // if(sensor1_time != 0 && sensor2_time != 0 && (readyToCalculate == true || readyToCalculate2 == true)) {
//   //   Serial.println("PODEMOS CALCULAR A VELOCIDADE");
//   //   Serial.print("Velocidade: ");
//   //   speed = double(200.0/(sensor2_time - sensor1_time));
//   //   Serial.println(speed);
//   //   if(measure1.RangeMilliMeter >= DETECTION_DISTANCE_MM) {
//   //     sensor1_time = 0;
//   //     dist1_ant = 1000;
//   //     readyToCalculate = false;
//   //   }
//   //   if(measure2.RangeMilliMeter >= DETECTION_DISTANCE_MM) {
//   //     sensor2_time = 0;
//   //     dist2_ant = 1000;
//   //     readyToCalculate2 = false;
//   //   }
//   // }

//   // print sensor one reading
//   // Serial.print(F("1: "));
//   // if(measure1.RangeStatus != 4) {     // if not out of range
//   //   Serial.print(measure1.RangeMilliMeter);
//   // } else {
//   //   Serial.print(F("Out of range"));
//   // }
  
//   // Serial.print(F(" "));

//   // // print sensor two reading
//   // Serial.print(F("2: "));
//   // if(measure2.RangeStatus != 4) {
//   //   Serial.print(measure2.RangeMilliMeter);
//   // } else {
//   //   Serial.print(F("Out of range"));
//   // }
  
//   // Serial.println();
// }

// void setup() {
//   Serial.begin(115200);

//   // wait until serial port opens for native USB devices
//   while (! Serial) { delay(1); }

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
//   delay(100);
// }
