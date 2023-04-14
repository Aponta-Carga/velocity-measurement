#include "Wire.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>


#define SENSOR_1_PIN 27
#define SENSOR_2_PIN 33

ros::NodeHandle nh;

std_msgs::Float32 speed_msg;
ros::Publisher speed_pub("sensor_data", &speed_msg);

long int sensor1_timestamp = 0, sensor2_timestamp = 0;
bool ready_to_calculate = false;
double speed = 0;

bool sensor_1_triggered = false, sensor_2_triggered = false;

void detect_sensor_changes() {
  if (!digitalRead(SENSOR_1_PIN) && !sensor_1_triggered) {
    sensor1_timestamp = micros();
    sensor_1_triggered = true;
  }
  
  if (!digitalRead(SENSOR_2_PIN) && !sensor_2_triggered) {
    sensor2_timestamp = micros();
    sensor_1_triggered = false;
    ready_to_calculate = true;
  }
}

void calculate_and_publish_speed() {
  if(ready_to_calculate) {
    speed = (200.0 / (sensor2_timestamp - sensor1_timestamp)) * 1000.0;
    Serial.println(speed);
    speed_msg.data = speed;
    speed_pub.publish(&speed_msg);
    nh.spinOnce();

    Serial.println(speed);
    if(!digitalRead(SENSOR_1_PIN)) {
      sensor_1_triggered = false;
    }
    if(!digitalRead(SENSOR_2_PIN)) {
      sensor_2_triggered = false;
      ready_to_calculate = false;
    }
  }
}

void setup() {
  nh.initNode();
  nh.advertise(speed_pub);

  Serial.begin(115200);

  while (!Serial) { delay(1); }

  pinMode(SENSOR_1_PIN, INPUT);
  pinMode(SENSOR_2_PIN, INPUT);

  Serial.println(F("Sensor's pins initialized..."));
  Serial.println(F("Starting..."));
}

void loop() {
  detect_sensor_changes();
  calculate_and_publish_speed();
  delay(100);
}