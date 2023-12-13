#include "Wire.h"
#include "HardwareSerial.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>




/*
 * Byte 1: Always is 0x59 
 * Byte 2: Distance in cm, low byte
 * Byte 3: Distance in cm, high byte
 * Byte 4: Signal strength, low byte
 * Byte 5: Signal strength, high byte
 * Byte 6: Reserved byte
 * Byte 7: Raw signal quality
 * Byte 8: Checksum of the 8 previous bytes.
 */


bool detectFrame( unsigned char *readBuffer ) {

  return  readBuffer[ 0 ] == 0x59 &&
          readBuffer[ 1 ] == 0x59 &&
          (unsigned char)(
            0x59 +
            0x59 +
            readBuffer[ 2 ] + 
            readBuffer[ 3 ] + 
            readBuffer[ 4 ] +
            readBuffer[ 5 ] + 
            readBuffer[ 6 ] + 
            readBuffer[ 7 ]
          ) == readBuffer[ 8 ];
}

unsigned int readLIDAR_B( long timeout ) {

  unsigned char readBuffer[ 9 ];

  long t0 = millis();

  while ( Serial1.available() < 9 ) {

    if ( millis() - t0 > timeout ) {
      // Timeout
      return 0;
    }

    delay( 10 );
  }

  for ( int i = 0; i < 9; i++ ) {
    readBuffer[ i ] = Serial1.read();
  }

  while ( ! detectFrame( readBuffer ) ) {

    if ( millis() - t0 > timeout ) {
      // Timeout
      return 0;
    }

    while ( Serial1.available() == 0 ) {
      delay( 10 );
    }

    for ( int i = 0; i < 8; i++ ) {
      readBuffer[ i ] = readBuffer[ i + 1 ];
    }

    readBuffer[ 8 ] = Serial1.read();

  }


  // Distance is in bytes 2 and 3 of the 9 byte frame.
  unsigned int distance = ( (unsigned int)( readBuffer[ 2 ] ) ) |
                          ( ( (unsigned int)( readBuffer[ 3 ] ) ) << 8 );

  return distance;

}

unsigned int readLIDAR( long timeout ) {

  unsigned char readBuffer[ 9 ];

  long t0 = millis();

  while ( Serial2.available() < 9 ) {

    if ( millis() - t0 > timeout ) {
      // Timeout
      return 0;
    }

    delay( 10 );
  }

  for ( int i = 0; i < 9; i++ ) {
    readBuffer[ i ] = Serial2.read();
  }

  while ( ! detectFrame( readBuffer ) ) {

    if ( millis() - t0 > timeout ) {
      // Timeout
      return 0;
    }

    while ( Serial2.available() == 0 ) {
      delay( 10 );
    }

    for ( int i = 0; i < 8; i++ ) {
      readBuffer[ i ] = readBuffer[ i + 1 ];
    }

    readBuffer[ 8 ] = Serial2.read();

  }


  // Distance is in bytes 2 and 3 of the 9 byte frame.
  unsigned int distance = ( (unsigned int)( readBuffer[ 2 ] ) ) |
                          ( ( (unsigned int)( readBuffer[ 3 ] ) ) << 8 );

  return distance;

}


void speedTest() {
/*
 * This function reads the Serial2 until a valid packet is found or timeout passed.
 * Param timeout: Timeout in milliseconds.
 * Returns distance in cm or 0 if timeout happened.
 */
  while ( Serial.available() > 0 ) {
    Serial.read();
  }

  Serial.printf( "\n\nPerforming speed test...\n" );

  long t0 = millis();

  #define NUM_READINGS 1000

  long accum = 0;

  for ( int i = 0; i < NUM_READINGS; i++ ) {

    accum += readLIDAR( 2000 );
  
  }

  long t1 = millis();

  float readingsPerSecond = NUM_READINGS * 1000.0f / ( t1 - t0 );

  float meanDistance = ((float)accum) / NUM_READINGS;

  Serial.println( "\n\nSpeed test:" );
  Serial.printf( "%f readings per second.\n", readingsPerSecond );
  Serial.printf( "%f mean read distance.\n", meanDistance );

  Serial.println( "\n\nHit another key to continue reading the sensor distance." );

  while ( Serial.available() == 0 ) {
    delay( 10 );
  }
  while ( Serial.available() > 0 ) {
    Serial.read();
  }

}

ros::NodeHandle nh;

std_msgs::Float32 distance_amsg;
ros::Publisher distance_apub("distance_a", &distance_amsg);


std_msgs::Float32 distance_bmsg;
ros::Publisher distance_bpub("distance_b", &distance_bmsg);


long int sensor1_timestamp = 0, sensor2_timestamp = 0;
bool ready_to_calculate = false;
double speed = 0;

bool sensor_1_triggered = false, sensor_2_triggered = false;
bool calculated_already = false;

unsigned int distance_a = 0;
unsigned int distance_b = 0;

int dist_a_threshold = 360;
int dist_b_threshold = 410;

float accumulated_distance_a = 0;
float accumulated_distance_b = 0;
int num_messages = 0;

float distance_c = 0;


void detect_sensor_changes() {
      if (num_messages >= 50) {
      // Calculate the mean of the accumulated distances
      float mean_distance_a = accumulated_distance_a / num_messages;
      float mean_distance_b = accumulated_distance_b / num_messages;

      // Subtract 20 from each mean distance to get the new thresholds
      dist_a_threshold = mean_distance_a - 20;
      dist_b_threshold = mean_distance_b - 20;

      // Calculate the square of distance_a and distance_b
      float square_distance_a = mean_distance_a * mean_distance_a;
      float square_distance_b = mean_distance_b * mean_distance_b;

      //std::cout << "dista: " << mean_distance_a << " km/h\n";
      //std::cout << "distb: " << mean_distance_b << " km/h\n";
      // Calculate the square of distance_c using the Pythagorean theorem
      float square_distance_c = square_distance_b - square_distance_a;

      // Calculate the distance_c by taking the square root of square_distance_c
      distance_c = sqrt(square_distance_c);
      //std::cout << "dist: " << distance_c << " km/h\n";

  if (distance_a > 500 && distance_b > 400)
    calculated_already = false;

  if (distance_a < 500 && distance_b > 400 && !sensor_1_triggered && !calculated_already) {
    sensor1_timestamp = micros();
    sensor_1_triggered = true;
    
  }
  if (distance_a > 500 && sensor_1_triggered) {
    sensor_1_triggered = false;
    calculated_already = false;
    
  }
  if (distance_b < 400 && !sensor_2_triggered && sensor_1_triggered && !calculated_already) {
    sensor2_timestamp = micros();
    ready_to_calculate = true;
    sensor_2_triggered = true;
    
  }
  if (distance_b > 400 && sensor_2_triggered) {
    sensor_2_triggered = false;

    // Publish "end" message
    //event_msg.data = 0;
    Serial.printf( "Distance A (cm): %d\n", distance_a );
    Serial.printf( "Distance B (cm): %d\n", distance_b );
    //Serial.println(event_msg.data);
    //event_pub.publish(&event_msg);
    nh.spinOnce();
  }
}

void calculate_and_publish_speed() {
  if (ready_to_calculate) {
    speed = (1550.0 / (sensor2_timestamp - sensor1_timestamp)) * 1000.0;
    Serial.printf( "Dist,ance A (cm): %d\n", distance_a );
    Serial.printf( "Distance B (cm): %d\n", distance_b );
    Serial.println(speed);
    //speed_msg.data = speed;
    //speed_pub.publish(&speed_msg);
    
    ready_to_calculate = false;
    
    //Serial.println(F("entered..."));
    // Publish "start" message
    //event_msg.data = 1;
    //Serial.println(event_msg.data);
    //event_pub.publish(&event_msg);
    nh.spinOnce();  
    calculated_already = true;
  }
}



void setup() {
  nh.initNode();
  //nh.advertise(speed_pub);
  //speed_msg.data = 1;
  // Advertise the event publisher
  //nh.advertise(event_pub);
  

  nh.advertise(distance_apub);
  nh.advertise(distance_bpub);

  // Debug serial
  Serial.begin( 115200 );

  // Serial connected to LIDAR sensor (UART 2)
  Serial1.begin(115200, SERIAL_8N1, 18); // RX: pin 16, TX: pin 17
  Serial2.begin(115200, SERIAL_8N1, 17); // RX: pin 16, TX: pin 17

  delay( 1000 );
  Serial.println( "Starting..." );

}

void loop() {


  
  
  // Perform one distance reading and show it on Serial
  distance_a = readLIDAR( 2000 );
  distance_b = readLIDAR_B( 2000 );

  Serial.printf( "Distance A (cm): %d\n", distance_a );
  Serial.printf( "Distance B (cm): %d\n", distance_b );
  distance_amsg.data = distance_a;
  distance_apub.publish(&distance_amsg);

  distance_bmsg.data = distance_b;
  distance_bpub.publish(&distance_bmsg);

  nh.spinOnce();  
      
    
  delay(10);
}



