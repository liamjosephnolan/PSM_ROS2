#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <Servo.h>

// Servo pins
#define SERVO1_PIN 9
#define SERVO2_PIN 10

// Hall effect sensor range
#define HALL_MIN 560
#define HALL_MAX 973

// Servo angle range (adjust these to match your grasper mechanism)
#define SERVO_CLOSED_ANGLE 0    // Angle when grasper is closed
#define SERVO_OPEN_ANGLE 90     // Angle when grasper is open

Servo servo1;
Servo servo2;

// Micro-ROS variables
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscriber;
std_msgs__msg__String msg;

// Error checking macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

float extract_sensor_value(const char* data, const char* sensor_name) {
  char search_str[20];
  snprintf(search_str, sizeof(search_str), "%s:", sensor_name);
  
  const char* pos = strstr(data, search_str);
  if (pos == NULL) return NAN;
  
  return atof(pos + strlen(search_str));
}

void control_grasper(float hall_value) {
  // Map hall value to servo angle (inverted if needed)
  float normalized = constrain(hall_value, HALL_MIN, HALL_MAX);
  float angle = map(normalized, HALL_MIN, HALL_MAX, SERVO_CLOSED_ANGLE, SERVO_OPEN_ANGLE);
  
  // Write to both servos
  servo1.write(angle);
  servo2.write(-angle);
  
  Serial.print("Hall: ");
  Serial.print(hall_value);
  Serial.print(" -> Servos: ");
  Serial.println(angle);
}

void subscription_callback(const void *msg_in) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
  
  float hall_value = extract_sensor_value(msg->data.data, "Hall");
  if (!isnan(hall_value)) {
    control_grasper(hall_value);
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Initialize to closed position
  control_grasper(HALL_MIN);
  delay(1000);

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "grasper_controller", "", &support));

  // Initialize message memory
  msg.data.data = (char*)malloc(200);
  msg.data.capacity = 200;
  msg.data.size = 0;

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/mtm_analog_values"));

  Serial.println("Grasper controller ready");
}

void loop() {
  static rclc_executor_t executor;
  static bool executor_initialized = false;
  
  if (!executor_initialized) {
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    executor_initialized = true;
  }
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}