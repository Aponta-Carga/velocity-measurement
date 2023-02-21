#include "Adafruit_VL53L1X.h"

// Defining new I2C adress
#define sensor1_iic_adress 0x30
#define sensor2_iic_adress 0x31


// Defining esp32 shutdown GPIOs
#define sensor1_XSHUT 7
#define sensor2_XSHUT 6


// Creating VL53l1X instances for both sensors
Adafruit_VL53L1X sensor1 = Adafruit_VL53L1X(sensor1_iic_adress);
Adafruit_VL53L1X sensor2 = Adafruit_VL53L1X(sensor2_iic_adress);


void setup() {
  // Initializing Serial
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (!Serial) delay(1);

  // Setting XSHUT pins to OUTPUT
  pinMode(sensor1_XSHUT, OUTPUT);
  pinMode(sensor2_XSHUT, OUTPUT);
  // assigning both sensors to reset mode
  digitalWrite(sensor1_XSHUT, LOW);
  digitalWrite(sensor2_XSHUT, LOW);
  delay(10);

  // Initializing wire library
  Wire.begin();


  // *---------- SENSOR 1 INITIALIZATION ----------*
  // activating sensor 1 and reseting sensor 2
  digitalWrite(sensor1_XSHUT, HIGH);
  digitalWrite(sensor2_XSHUT, LOW);
  // initializing sensor 1
  if (! sensor1.begin(sensor1_iic_adress, &Wire)) {
    Serial.print(F("Error on init of VL sensor 1: "));
    Serial.println(sensor1.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor 1 OK!"));
  // printing sensor 1 i2c id
  Serial.print(F("Sensor ID: 0x"));
  Serial.println(sensor1.sensorID(), HEX);
  // checking if sensor 1 is ranging
  if (! sensor1.startRanging()) {
    Serial.print(F("Couldn't start sensor 1 ranging: "));
    Serial.println(sensor1.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Sensor 1 ranging started"));
  sensor1.setTimingBudget(20);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(sensor1.getTimingBudget());


  // *---------- SENSOR 2 INITIALIZATION ----------*
  // activating sensor 2
  digitalWrite(sensor2_XSHUT, HIGH);
  // initializing sensor 2
  if (! sensor2.begin(sensor2_iic_adress, &Wire)) {
    Serial.print(F("Error on init of VL sensor 2: "));
    Serial.println(sensor2.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor 2 OK!"));
  // printing sensor 2 i2c id
  Serial.print(F("Sensor ID: 0x"));
  Serial.println(sensor2.sensorID(), HEX);
  // checking if sensor 2 is ranging
  if (! sensor2.startRanging()) {
    Serial.print(F("Couldn't start sensor 2 ranging: "));
    Serial.println(sensor2.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Sensor 2 ranging started"));
  sensor2.setTimingBudget(20);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(sensor2.getTimingBudget());
}


float measure_velocity_mps() {
  int16_t sensor1_distance = 0, sensor2_distance = 0;
  unsigned long truck_enter_time = 0, truck_leave_time = 0 ;

  if (sensor1.dataReady() && sensor2.dataReady()) {
    // getting sensor 1 distance reading
    sensor1_distance = sensor1.distance();
    if (sensor1_distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(sensor1.vl_status);
      return 0;
    }

    // if sensor 1 distance is smaller than 3 meters, there is a truck passing under
    // so the current time must be recorded
    if(sensor1_distance < 3000) {
      truck_enter_time = millis();
    }

    sensor2_distance = sensor2.distance();
    if (sensor2_distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(sensor2.vl_status);
      return 0;
    }

    // while the heigth diference between sensor 1 distance and current sensor 2 
    // distance reading is smaller than 10 cm the truck hasn't reached sensor 2
    while(abs(sensor2_distance - sensor1_distance) > 100) {
      sensor2_distance = sensor2.distance();
      if (sensor2_distance == -1) {
        // something went wrong!
        Serial.print(F("Couldn't get distance: "));
        Serial.println(sensor2.vl_status);
        return 0;
      }
      delay(10);
    }
    
    truck_leave_time = millis();

  }
  // Considering that the distance between sensor is 10cm 
  // the velocity can be calculated accordingly
  return (100/(truck_leave_time - truck_enter_time))*1000;
}


void loop() {
  float truck_velocity = measure_velocity_mps(); // measure the velocity of an object under the two sensors in meters per seconds
}
