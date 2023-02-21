import time
import board
import digitalio
import adafruit_vl53l1x

# Define the I2C pins.
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

xshut = [
    # Update the D6 and D5 pins to match the pins to which you wired your sensor XSHUT pins.
    digitalio.DigitalInOut(board.D6),
    digitalio.DigitalInOut(board.D5),
    # Add more VL53L1X sensors by defining their XSHUT pins here.
]

for shutdown_pin in xshut:
    # Set the shutdown pins to output, and pull them low.
    shutdown_pin.switch_to_output(value=False)
    # These pins are active when Low, meaning:
    #   If the output signal is LOW, then the VL53L1X sensor is off.
    #   If the output signal is HIGH, then the VL53L1X sensor is on.
# All VL53L1X sensors are now off.

# Create a list to be used for the array of VL53L1X sensors.
vl53l1x = []

# Change the address of the additional VL53L1X sensors.
for pin_number, shutdown_pin in enumerate(xshut):
    # Turn on the VL53L1X sensors to allow hardware check.
    shutdown_pin.value = True
    # Instantiate the VL53L1X I2C object and insert it into the vl53l1x list.
    # This also performs VL53L1X hardware check.
    sensor_i2c = adafruit_vl53l1x.VL53L1X(i2c)
    vl53l1x.append(sensor_i2c)
    # This ensures no address change on one sensor board, specifically the last one in the series.
    if pin_number < len(xshut) - 1:
        # The default address is 0x29. Update it to an address that is not already in use.
        sensor_i2c.set_address(pin_number + 0x30)

# Print the various sensor I2C addresses to the serial console.
if i2c.try_lock():
    print("Sensor I2C addresses:", [hex(x) for x in i2c.scan()])
    i2c.unlock()

# Start ranging for sensor data collection.
for sensor in vl53l1x:
    sensor.start_ranging()


# This function should measure the velocity in meters per second based on the elapsed time.
def measure_speed_mps():
  if vl53l1x[0].sensor.data_ready and vl53l1x[1].sensor.data_ready:
      sensor1_distance = vl53l1x[0].sensor.distance

      if sensor1_distance < 3000:
        truck_enter_time = time()
      
        sensor2_distance = vl53l1x[1].sensor.distance

        while(abs(sensor2_distance - sensor1_distance) > 10):
          sensor2_distance = vl53l1x[1].sensor.distance
          time.sleep(10/1000) # sleeps for 10 milliseocnds

        truck_leave_time = time();

  return (100/(truck_leave_time - truck_enter_time))*1000;


while True:
    speed = measure_speed_mps()
    time.sleep(0.5)