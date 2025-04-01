#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h> // Use String message type

#include <Servo.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Define Servo pin
#define SERVO1 9

// Servo object
Servo servo1;

// Servo test variables
int pos = 0;            // current position
int increment = 1;      // increment to move
unsigned long prevTime = 0;
const int interval = 15; // ms between movements

void error_loop() {
  while (1) {
    delay(100);
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000); // Initial delay for micro-ROS setup

  // Attach servo to pin
  servo1.attach(SERVO1);
  
  // Initialize servo to 0 position
  servo1.write(0);
  delay(1000); // Give servo time to reach position
}

void loop() {
  // Servo sweep test
  unsigned long currentTime = millis();
  
  if (currentTime - prevTime >= interval) {
    prevTime = currentTime;
    
    // Update position
    pos += increment;
    servo1.write(pos);
    
    // Reverse direction at limits
    if (pos >= 180 || pos <= 0) {
      increment = -increment;
    }
  }
  
  // You can add micro-ROS node spinning here when ready
  // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}