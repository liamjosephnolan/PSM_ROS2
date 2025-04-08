#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// ROS 2
rcl_subscription_t joint_state_subscriber;
sensor_msgs__msg__JointState received_joint_state;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Servo
Servo gimbal3_servo;
#define SERVO_PIN 12

// Error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) { delay(100); }
}

void joint_state_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;

  if (msg->position.size > 3) {
    double g3_deg = msg->position.data[3];  // right_gimbal_3 is index 3

    // Map degrees (-125° to 125°) to servo angle (0–180)
    int servo_angle = map(g3_deg, -125, 125, 0, 180);
    servo_angle = constrain(servo_angle, 0, 180);

    gimbal3_servo.write(servo_angle);
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  gimbal3_servo.attach(SERVO_PIN);
  gimbal3_servo.write(90);  // Set to neutral position
  delay(1000);  // Wait for servo to reach position

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "gimbal3_servo_node", "", &support));

  // Initialize subscriber
  RCCHECK(rclc_subscription_init_default(
    &joint_state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/mtm_joint_states"));

  // Prepare message memory
  received_joint_state.position.data = (double*) malloc(7 * sizeof(double));
  received_joint_state.position.size = 7;
  received_joint_state.position.capacity = 7;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &joint_state_subscriber, &received_joint_state, &joint_state_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
  gimbal3_servo.write(random(0,360));  // Set to neutral position
  delay(1000);  // Wait for servo to reach position
}
