#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>

// === Configuration ===
#define SERVO_PIN 12
#define GIMBAL_JOINT_INDEX 3  // right_gimbal_3
#define JOINT_NAME_MAX 10     // Max joints expected
#define NAME_LENGTH_MAX 30    // Max length of joint name
#define GIMBAL_DEG_MIN -125
#define GIMBAL_DEG_MAX 125
#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180

// === ROS 2 Stuff ===
rcl_subscription_t joint_state_subscriber;
sensor_msgs__msg__JointState received_joint_state;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// === Servo Control ===
Servo gimbal3_servo;

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

  if (msg->position.size > GIMBAL_JOINT_INDEX) {
    double g3 = msg->position.data[GIMBAL_JOINT_INDEX];
    int servo_angle = map_gimbal_to_servo(g3);

    gimbal3_servo.write(servo_angle);

    // Debug output
    Serial.print("right_gimbal_3: ");
    Serial.print(g3, 2);
    Serial.print(" deg -> Servo angle: ");
    Serial.println(servo_angle);
  } else {
    Serial.println("Warning: Not enough joint positions received.");
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  Serial.println("Initializing gimbal servo node...");

  gimbal3_servo.attach(SERVO_PIN);
  gimbal3_servo.write(90);  // Neutral
  delay(500);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "gimbal3_servo_node", "", &support));

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
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}
