#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include <Arduino.h>

// -----> PUB --------------------------------------   
#define speed_topic "speed_controller"

// ----- PUB MSG ------------------------------------
std_msgs::Float32 speedMsg;
ros::Publisher speedSub(speed_topic, &speedMsg); 

//* <------------ ROS setup ---------->
ros::NodeHandle  nh;

void ros_init(){

    nh.initNode(); 
    nh.advertise(speedSub); 
    
}

void ros_loop(float speed) {
    
    speedMsg.data = speed; 
    speedSub.publish(&speedMsg); 
    nh.spinOnce();
  
}