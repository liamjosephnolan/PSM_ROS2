#include <Arduino.h>
#include <Servo.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>

// Define the servo pin
#define SERVO4_PIN 12 // Roll

// Create a Servo object
Servo servo4;

// Micro-ROS variables
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscriber;
rcl_publisher_t debug_publisher; // Debug publisher for servo position
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__JointState debug_msg; // Debug message for publishing servo position

// Error checking macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

// Callback function to process joint state messages
void joint_state_callback(const void *msg_in) {
  Serial1.println("Callback triggered for /mtm_joint_states");

  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msg_in;

  for (size_t i = 0; i < msg->name.size; i++) {
    if (strcmp(msg->name.data[i].data, "right_gimbal_3") == 0) {
      float g3_position = msg->position.data[i];
      Serial1.print("G3 Position: ");
      Serial1.println(g3_position);

      int servo_position = map(g3_position, -125, 125, 0, 180);
      servo_position = constrain(servo_position, 0, 180);

      servo4.write(servo_position);

      debug_msg.position.data[0] = servo_position;
      RCCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

      Serial1.print("Published servo position: ");
      Serial1.println(servo_position);

      break;
    }
  }
}

void setup() {
  // Initialize Serial communication for micro-ROS
  Serial.begin(115200); // For micro-ROS communication
  set_microros_serial_transports(Serial);
  delay(2000);

  // Initialize Serial1 for debugging
  Serial1.begin(9600); // For debugging output
  delay(2000);

  // Attach the servo to the specified pin
  servo4.attach(SERVO4_PIN);

  // Initialize servo to 0 degrees
  servo4.write(0);
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

  // Create publisher for debugging servo position
  RCCHECK(rclc_publisher_init_default(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/servo4_debug"));

  // Initialize debug message
  debug_msg.position.data = (double *)malloc(1 * sizeof(double));
  debug_msg.position.size = 1;
  debug_msg.position.capacity = 1;

  // Test publishing a message
  debug_msg.position.data[0] = 90.0; // Example position
  RCCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
  Serial1.println("Test message published to /servo4_debug");

}

void loop() {
  Serial1.println("Looping...");
  // Spin the executor to process incoming messages
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
