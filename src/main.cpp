#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <Servo.h>

#define SERVO_PIN_1 9
#define SERVO_PIN_2 10
#define SERVO_PIN_3 11
#define SERVO_PIN_4 12

Servo servo1, servo2, servo3, servo4;
rcl_subscription_t joint_state_subscriber;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

sensor_msgs__msg__JointState joint_state_msg;

int servo_off[4] = {105, 90, 90, 90};  // Offsets for each servo
int servo_val[4];

void error_loop(const char *file, int line) {
  while (1) {
    Serial.print("Error in setup, entering error loop. File: ");
    Serial.print(file);
    Serial.print(", Line: ");
    Serial.println(line);
    delay(1000); // Wait 1 second before repeating
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(__FILE__, __LINE__);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.print("Soft error at "); Serial.print(__FILE__); Serial.print(":"); Serial.println(__LINE__);}}

void joint_state_callback(const void *msgin) {
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

  // Check that we received enough data
  if (msg->position.size < 7 || msg->position.data == NULL) {
    // If data is invalid, pulse servos to 90 to indicate error
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);
    return;
  }
  
  // Extract joint values (in degrees)
  float g0 = msg->position.data[6]; // right_gimbal_0
  float g1 = msg->position.data[5]; // right_gimbal_1
  float g2 = msg->position.data[4]; // right_gimbal_2
  float g3 = msg->position.data[3]; // right_gimbal_3

  // Compute servo positions
  float s1 = -g0 / 1.4 + (servo_off[0] - 90);
  float s2 =  g1 / 1.0 + (servo_off[1] - 90);
  float s3 = (2 * g2 - g3 + 1.5 * s2) / 2.0 + (servo_off[2] - 90);
  float s4 = g3 / 1.0 + s3 + (servo_off[3] - 90);

  // Map and constrain to [0, 180]
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

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Starting setup...");
  delay(2000);
  
  Serial1.begin(921600);
  set_microros_serial_transports(Serial1);
  delay(2000);

  servo1.attach(SERVO_PIN_1);
  servo2.attach(SERVO_PIN_2);
  servo3.attach(SERVO_PIN_3);
  servo4.attach(SERVO_PIN_4);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "servo_controller_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &joint_state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "mtm_joint_states"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &joint_state_subscriber,
    &joint_state_msg,
    &joint_state_callback,
    ON_NEW_DATA));

    sensor_msgs__msg__JointState__init(&joint_state_msg);
}

void loop() {
  Serial.println("Looping...");

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(5);  // Adjust delay as needed
}


