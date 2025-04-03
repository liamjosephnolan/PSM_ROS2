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
#ifdef PSM_RIGHT
const int servo_offsets[4] = {115, 92, 80, 105};  // Right PSM offsets
#else
const int servo_offsets[4] = {105, 90, 90, 90};   // Left PSM offsets
#endif

// Micro-ROS variables
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState joint_state_msg;

// Error checking macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

// Kinematic mapping functions
void update_servos_from_joint_state(const sensor_msgs__msg__JointState * msg) {
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
  // Note: These mappings may need adjustment based on your specific mechanism
  
  // Wrist roll (servo3) - maps directly to gimbal_3 (-125° to +125°)
  int servo3_pos = servo_offsets[2] + map(g3, -125, 125, -90, 90);
  
  // Wrist pitch (servo4) - maps to gimbal_2 (-40° to +40°)
  int servo4_pos = servo_offsets[3] + map(g2, -40, 40, -45, 45);
  
  // Grasper opening (servo1 and servo2) - based on gimbal_1 (-40° to +40°)
  // The 0.67 factor accounts for mechanical coupling between yaw and finger motion
  int finger_offset = map(g1, -40, 40, -60, 60) + 0.67 * (servo3_pos - servo_offsets[2]);
  
  // Add the gripper open/close (g0: 0-20°)
  int grip_open = map(g0, 0, 20, 0, 60); // 0=closed, 60=open
  
  // Right finger (servo1)
  int servo1_pos = servo_offsets[0] + finger_offset + grip_open/2;
  
  // Left finger (servo2) - moves in opposite direction
  int servo2_pos = servo_offsets[1] + finger_offset - grip_open/2;
  
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
  
  // Debug output
  /*
  Serial.print("G0:"); Serial.print(g0);
  Serial.print(" G1:"); Serial.print(g1);
  Serial.print(" G2:"); Serial.print(g2);
  Serial.print(" G3:"); Serial.print(g3);
  Serial.print(" | Servos:");
  Serial.print(servo1_pos); Serial.print(",");
  Serial.print(servo2_pos); Serial.print(",");
  Serial.print(servo3_pos); Serial.print(",");
  Serial.println(servo4_pos);
  */
}

void joint_state_callback(const void *msg_in) {
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msg_in;
  update_servos_from_joint_state(msg);
}

void setup() {
  Serial.begin(115200);
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

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "psm_servo_controller", "", &support));

  // Create subscriber for joint states
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/mtm_joint_states"));

  Serial.println("PSM Servo Controller ready");
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