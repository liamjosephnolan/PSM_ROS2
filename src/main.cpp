#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Servo.h>
#include <Encoder.h>
#include "CytronMotorDriver.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include "encoder_reader.h"
#include "config.h"

// Declare the home_motors function defined in homing.cpp
extern void home_motors();

// === ROS 2 Stuff ===
rcl_subscription_t joint_state_subscriber;
rcl_publisher_t sensor_data_publisher;
rcl_publisher_t debug_publisher;
sensor_msgs__msg__JointState received_joint_state;
std_msgs__msg__Int32MultiArray sensor_data_msg;
std_msgs__msg__String debug_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// === Servo Control ===
Servo servo1, servo2, servo3, servo4;

// Motor driver configuration
CytronMD motor1(PWM_DIR, DC1_PWM, DC1_DIR);
CytronMD motor2(PWM_DIR, DC2_PWM, DC2_DIR);
CytronMD motor3(PWM_DIR, DC3_PWM, DC3_DIR);
CytronMD motor[3] = { motor1, motor2, motor3 };

// Servo offsets (calibration values)
int servo_off1 = 100, servo_off2 = 97, servo_off3 = 90, servo_off4 = 92;
int servo_off[4] = { servo_off1, servo_off2, servo_off3, servo_off4 };

// Servo values
int servo_val[4] = {0, 0, 0, 0};

// === Macros ===
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {} }

// === Debug Message Function ===
void publish_debug_message(const char *message) {
  snprintf(debug_msg.data.data, debug_msg.data.capacity, "%s", message);
  debug_msg.data.size = strlen(debug_msg.data.data);
  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
}

// === Error Handling ===
void error_loop() {
  while (1) {
    publish_debug_message("Error occurred!");
    delay(1000);
  }
}

// === Servo Mapping ===
int map_gimbal_to_servo(double gimbal_angle, double gimbal_min, double gimbal_max, double max_angle, double min_angle) {
  int servo_angle = map((int)gimbal_angle, gimbal_min, gimbal_max, max_angle, min_angle);
  return constrain(servo_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
}

//TODO: Homing sequence for motors

// === Callback ===
void joint_state_callback(const void *msgin) {
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

  if (msg->position.size < 7 || msg->position.data == NULL) {
    publish_debug_message("[WARN] Invalid joint state message.");
    return;
  }

  // Extract joint positions
  float g0 = msg->position.data[6]; // Grasper open/close (0–90)
  float g1 = msg->position.data[5]; // Wrist tilt (-40 to 40)
  float g2 = msg->position.data[4]; // Grasper tilt (-40 to 40)
  float g3 = msg->position.data[3]; // Roll (-125 to 125)

  // Map gimbal angles to servo angles
  g3 = map_gimbal_to_servo(g3, G3_MIN, G3_MAX, -180, 180); // Map roll to servo range
  g2 = map_gimbal_to_servo(g2, G2_MIN, G2_MAX, -90, 90);   // Map grasper tilt to servo range
  g1 = map_gimbal_to_servo(g1, G1_MIN, G1_MAX, -90, 90);   // Map wrist tilt to servo range
  g0 = map_gimbal_to_servo(g0, G0_MIN, G0_MAX, -90, 90);   // Map grasper open/close to servo range

  // Map and constrain servo values
  servo_val[0] = -g0 / 2 + g1 + servo_off[0];
  servo_val[1] = g0 / 2 + g1 + servo_off[1];
  servo_val[2] = g2 + servo_off[2];
  servo_val[3] = g3 + servo_off[3];

  // Write to servos
  servo1.write(servo_val[0]);
  servo2.write(servo_val[1]);
  servo3.write(servo_val[2]);
  servo4.write(servo_val[3]);
}

// Function to check limit switch states and publish their status
void publish_limit_switch_status() {
  // Read the state of each limit switch
  int limit_switch_states[3];
  limit_switch_states[0] = digitalRead(LS1_NC); // Limit switch 1
  limit_switch_states[1] = digitalRead(LS2_NC); // Limit switch 2
  limit_switch_states[2] = digitalRead(LS3_NC); // Limit switch 3

  // Publish debug messages for triggered limit switches
  char debug_msg[128];
  if (limit_switch_states[0] == HIGH) {
    snprintf(debug_msg, sizeof(debug_msg), "Limit Switch 1 triggered!");
    publish_debug_message(debug_msg);
  }
  if (limit_switch_states[1] == HIGH) {
    snprintf(debug_msg, sizeof(debug_msg), "Limit Switch 2 triggered!");
    publish_debug_message(debug_msg);
  }
  if (limit_switch_states[2] == HIGH) {
    snprintf(debug_msg, sizeof(debug_msg), "Limit Switch 3 triggered!");
    publish_debug_message(debug_msg);
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Configure limit switch pins as inputs
  pinMode(LS1_NO, INPUT_PULLUP); // Normally open pin for Limit Switch 1
  pinMode(LS2_NO, INPUT_PULLUP); // Normally open pin for Limit Switch 2
  pinMode(LS3_NO, INPUT_PULLUP); // Normally open pin for Limit Switch 3

  pinMode(LS1_NC, INPUT_PULLUP); // Normally closed pin for Limit Switch 1
  pinMode(LS2_NC, INPUT_PULLUP); // Normally closed pin for Limit Switch 2
  pinMode(LS3_NC, INPUT_PULLUP); // Normally closed pin for Limit Switch 3

  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  // Initialize servos to neutral positions
  servo1.write(servo_off[0]);
  servo2.write(servo_off[1]);
  servo3.write(servo_off[2]);
  servo4.write(servo_off[3]);
  delay(1000);

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "psm_sensor_node", "", &support));

  // Create subscriber for joint states
  RCCHECK(rclc_subscription_init_default(
    &joint_state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/mtm_joint_states"));

  // Create publisher for sensor data
  RCCHECK(rclc_publisher_init_default(
    &sensor_data_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/psm_sensor_data"));

  // Create publisher for debug messages
  RCCHECK(rclc_publisher_init_default(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/psm_debug"));

  // Initialize sensor data message
  sensor_data_msg.data.data = (int32_t *)malloc(3 * sizeof(int32_t)); // 3 encoders
  sensor_data_msg.data.size = 3;
  sensor_data_msg.data.capacity = 3;

  // Initialize debug message
  debug_msg.data.data = (char *)malloc(128 * sizeof(char)); // Allocate space for debug messages
  debug_msg.data.size = 0;
  debug_msg.data.capacity = 128;

  // === Init message fields ===
  memset(&received_joint_state, 0, sizeof(sensor_msgs__msg__JointState));

  // Allocate joint names
  received_joint_state.name.data = (rosidl_runtime_c__String *)malloc(JOINT_NAME_MAX * sizeof(rosidl_runtime_c__String));
  received_joint_state.name.size = JOINT_NAME_MAX;
  received_joint_state.name.capacity = JOINT_NAME_MAX;

  for (size_t i = 0; i < received_joint_state.name.size; i++) {
    rosidl_runtime_c__String__init(&received_joint_state.name.data[i]);
    received_joint_state.name.data[i].data = (char *)malloc(NAME_LENGTH_MAX);
    received_joint_state.name.data[i].size = 0;
    received_joint_state.name.data[i].capacity = NAME_LENGTH_MAX;
  }

  // Allocate joint positions
  received_joint_state.position.data = (double *)malloc(JOINT_NAME_MAX * sizeof(double));
  received_joint_state.position.size = JOINT_NAME_MAX;
  received_joint_state.position.capacity = JOINT_NAME_MAX;

  // Skip velocity and effort for now (zero-length)
  received_joint_state.velocity.data = NULL;
  received_joint_state.velocity.size = 0;
  received_joint_state.velocity.capacity = 0;
  received_joint_state.effort.data = NULL;
  received_joint_state.effort.size = 0;
  received_joint_state.effort.capacity = 0;

  // === Executor ===
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &joint_state_subscriber,
    &received_joint_state,
    &joint_state_callback,
    ON_NEW_DATA));

  // Home all motors
  home_motors(); 

  // Publish debug message
  publish_debug_message("PSM Sensor Node ready and listening!");

}

// === Loop ===
void loop() {
  // Publish encoder data
  sensor_data_msg.data.data[0] = Enc1.read();
  sensor_data_msg.data.data[1] = Enc2.read();
  sensor_data_msg.data.data[2] = Enc3.read();
  RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));



  // Spin executor
  RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));
  delay(5);
}
