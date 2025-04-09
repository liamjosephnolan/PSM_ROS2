#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>

// === Configuration ===
#define SERVO1_PIN 9  // Right grasper finger
#define SERVO2_PIN 10 // Left grasper finger
#define SERVO3_PIN 11 // Wrist roll
#define SERVO4_PIN 12 // Wrist pitch

#define GIMBAL_DEG_MIN -125
#define GIMBAL_DEG_MAX 125
#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180

#define JOINT_NAME_MAX 10
#define NAME_LENGTH_MAX 30

// === ROS 2 Stuff ===
rcl_subscription_t joint_state_subscriber;
sensor_msgs__msg__JointState received_joint_state;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// === Servo Control ===
Servo servo1, servo2, servo3, servo4;

// Servo offsets (calibration values)
float servo_off[4] = {90, 90, 90, 90}; // Adjust these based on your setup

// Servo values
int servo_val[4] = {0, 0, 0, 0};

// === Macros ===
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {} }

void error_loop() {
  while (1) {
    Serial.println("Error occurred. Looping...");
    delay(1000);
  }
}

// === Servo Mapping ===
int map_gimbal_to_servo(double gimbal_angle) {
  int servo_angle = map((int)gimbal_angle, GIMBAL_DEG_MIN, GIMBAL_DEG_MAX, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
  return constrain(servo_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
}

// === Callback ===
void joint_state_callback(const void *msgin) {
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

  if (msg->position.size < 7 || msg->position.data == NULL) {
    Serial.println("[WARN] Invalid joint state message.");
    return;
  }

  // Extract joint positions
  float g0 = msg->position.data[6]; // right_gimbal_0
  float g1 = msg->position.data[5]; // right_gimbal_1
  float g2 = msg->position.data[4]; // right_gimbal_2
  float g3 = msg->position.data[3]; // right_gimbal_3

  // Calculate servo positions
  float s1 = -g0 / 1.4 + (servo_off[0] - 90);
  float s2 = g1 / 1.0 + (servo_off[1] - 90);
  float s3 = (2 * g2 - g3 + 1.5 * s2) / 2.0 + (servo_off[2] - 90);
  float s4 = g3 + s3 + (servo_off[3] - 90);

  // Map and constrain servo values
  servo_val[0] = constrain(map(s1, -90, 90, 0, 180), 0, 180);
  servo_val[1] = constrain(map(s2, -90, 90, 0, 180), 0, 180);
  servo_val[2] = constrain(map(s3, -90, 90, 0, 180), 0, 180);
  servo_val[3] = constrain(map(s4, -90, 90, 0, 180), 0, 180);

  // Write to servos
  servo1.write(servo_val[0]);
  servo2.write(servo_val[1]);
  servo3.write(servo_val[2]);
  servo4.write(servo_val[3]);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

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
  RCCHECK(rclc_node_init_default(&node, "gimbal3_servo_node", "", &support));

  // Create subscriber for joint states
  RCCHECK(rclc_subscription_init_default(
    &joint_state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/mtm_joint_states"));

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

  Serial.println("Gimbal3 servo node ready and listening!");
}

// === Loop ===
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));
}
