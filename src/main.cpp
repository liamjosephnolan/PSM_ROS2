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

// Gimbal angle ranges (adjustable)
#define G0_MIN 0 //TODO Fix G0 angle stuff
#define G0_MAX 75
#define G1_MIN -40
#define G1_MAX 40
#define G2_MIN -40
#define G2_MAX 40
#define G3_MIN -125 
#define G3_MAX 125

// Servo angle range
#define SERVO_ANGLE_MIN -360
#define SERVO_ANGLE_MAX 360

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
int servo_off1 = 100, servo_off2 = 97, servo_off3 = 90, servo_off4 = 92;
int servo_off[4] = { servo_off1, servo_off2, servo_off3, servo_off4 };

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
int map_gimbal_to_servo(double gimbal_angle, double gimbal_min, double gimbal_max, double max_angle, double min_angle) {
  int servo_angle = map((int)gimbal_angle, gimbal_min, gimbal_max, max_angle, min_angle);
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
  float g0 = msg->position.data[6]; // Grasper open/close (0–90)
  float g1 = msg->position.data[5]; // Wrist tilt (-40 to 40)
  float g2 = msg->position.data[4]; // Grasper tilt (-40 to 40)
  float g3 = msg->position.data[3]; // Roll (-125 to 125)

  g3 = map_gimbal_to_servo(g3, G3_MIN, G0_MAX, -180, 180); // Map roll to servo range
  g2 = map_gimbal_to_servo(g2, G2_MIN, G2_MAX, -90, 90); // Map grasper tilt to servo range
  g1 = map_gimbal_to_servo(g1, G1_MIN, G1_MAX, -90, 90); // Map wrist tilt to servo range
  g0 = map_gimbal_to_servo(g0, G0_MIN, G0_MAX, -90, 90);  // Map grasper open/close to servo range

  // Map and constrain servo values
  servo_val[0] = -g0/2 + g1 + servo_off[0];
  servo_val[1] = g0/2 + g1 + servo_off[1];
  servo_val[2] = g2 + servo_off[2];
  servo_val[3] = g3 + servo_off[3];

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
