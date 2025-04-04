#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <Servo.h>

// Servo pins
#define SERVO1_PIN 9  // Right grasper finger
#define SERVO2_PIN 10 // Left grasper finger
#define SERVO3_PIN 11 // Wrist roll
#define SERVO4_PIN 12 // Wrist pitch

// Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Servo calibration offsets (adjust these based on your mechanical setup)
const int servo_offsets[4] = {115, 92, 80, 105};  // Right PSM offsets

// Micro-ROS variables
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState joint_state_msg;

// Add a publisher for servo positions
rcl_publisher_t servo_position_publisher;
sensor_msgs__msg__JointState servo_position_msg;

// Error checking macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

void update_servos_from_joint_state(const sensor_msgs__msg__JointState *msg) {
  // Extract joint positions (in degrees)
  float g0 = 0, g1 = 0, g2 = 0, g3 = 0;

  // Find the joint values in the message
  for (size_t i = 0; i < msg->name.size; i++) {
    if (strcmp(msg->name.data[i].data, "right_gimbal_0") == 0) {
      g0 = msg->position.data[i];
    } else if (strcmp(msg->name.data[i].data, "right_gimbal_1") == 0) {
      g1 = msg->position.data[i];
    } else if (strcmp(msg->name.data[i].data, "right_gimbal_2") == 0) {
      g2 = msg->position.data[i];
    } else if (strcmp(msg->name.data[i].data, "right_gimbal_3") == 0) {
      g3 = msg->position.data[i];
    }
  }

  // Calculate servo positions based on kinematics
  int servo3_pos = servo_offsets[2] + map(g3, -125, 125, -90, 90);
  int servo4_pos = servo_offsets[3] + map(g2, -40, 40, -45, 45);
  int finger_offset = map(g1, -40, 40, -60, 60) + 0.67 * (servo3_pos - servo_offsets[2]);
  int grip_open = map(g0, 0, 20, 0, 60);

  int servo1_pos = servo_offsets[0] + finger_offset + grip_open / 2;
  int servo2_pos = servo_offsets[1] + finger_offset - grip_open / 2;

  // Constrain to valid servo ranges
  servo1_pos = constrain(servo1_pos, 0, 180);
  servo2_pos = constrain(servo2_pos, 0, 180);
  servo3_pos = constrain(servo3_pos, 0, 180);
  servo4_pos = constrain(servo4_pos, 0, 180);

  // Write to servos
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);
  servo4.write(servo4_pos);

  // Publish servo positions
  servo_position_msg.position.data[0] = servo1_pos;
  servo_position_msg.position.data[1] = servo2_pos;
  servo_position_msg.position.data[2] = servo3_pos;
  servo_position_msg.position.data[3] = servo4_pos;

  RCCHECK(rcl_publish(&servo_position_publisher, &servo_position_msg, NULL));
}

void joint_state_callback(const void *msg_in) {
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msg_in;
  update_servos_from_joint_state(msg);
}

void setup() {
  // Initialize micro-ROS
  set_microros_serial_transports(Serial);
  delay(2000);

  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  // Initialize to neutral position
  servo1.write(servo_offsets[0]);
  servo2.write(servo_offsets[1]);
  servo3.write(servo_offsets[2]);
  servo4.write(servo_offsets[3]);
  delay(1000);
  servo4.write(0);
  delay(1000);
  servo4.write(180);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "psm_servo_controller", "", &support));

  // Create subscriber for joint states
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/mtm_joint_states"));

  // Create publisher for servo positions
  RCCHECK(rclc_publisher_init_default(
    &servo_position_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/servo_positions"));

  // Initialize the servo position message
  servo_position_msg.name.data = (rosidl_runtime_c__String *)malloc(4 * sizeof(rosidl_runtime_c__String));
  servo_position_msg.name.size = 4;
  servo_position_msg.name.capacity = 4;
  servo_position_msg.position.data = (double *)malloc(4 * sizeof(double));
  servo_position_msg.position.size = 4;
  servo_position_msg.position.capacity = 4;

  // Manually assign names to the servo_position_msg
  servo_position_msg.name.data[0].data = strdup("servo1");
  servo_position_msg.name.data[0].size = strlen("servo1");
  servo_position_msg.name.data[0].capacity = strlen("servo1") + 1;

  servo_position_msg.name.data[1].data = strdup("servo2");
  servo_position_msg.name.data[1].size = strlen("servo2");
  servo_position_msg.name.data[1].capacity = strlen("servo2") + 1;

  servo_position_msg.name.data[2].data = strdup("servo3");
  servo_position_msg.name.data[2].size = strlen("servo3");
  servo_position_msg.name.data[2].capacity = strlen("servo3") + 1;

  servo_position_msg.name.data[3].data = strdup("servo4");
  servo_position_msg.name.data[3].size = strlen("servo4");
  servo_position_msg.name.data[3].capacity = strlen("servo4") + 1;
}

void loop() {
  static rclc_executor_t executor;
  static bool executor_initialized = false;

  if (!executor_initialized) {
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &joint_state_msg, &joint_state_callback, ON_NEW_DATA));
    executor_initialized = true;
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}