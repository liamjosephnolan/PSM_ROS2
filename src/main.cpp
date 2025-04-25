#include <Arduino.h>
#include "config.h"

// ----------------------
// Global ROS 2 Objects
// ----------------------
rcl_subscription_t joint_state_subscriber;
rcl_subscription_t target_pose_subscriber;
rcl_publisher_t sensor_data_publisher;
rcl_publisher_t debug_publisher;

sensor_msgs__msg__JointState received_joint_state;
geometry_msgs__msg__PoseStamped target_pose_msg;
std_msgs__msg__Int32MultiArray sensor_data_msg;
std_msgs__msg__String debug_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ----------------------
// Servo and Motor Control
// ----------------------
Servo servo1, servo2, servo3, servo4;

CytronMD motor1(PWM_DIR, DC1_PWM, DC1_DIR);
CytronMD motor2(PWM_DIR, DC2_PWM, DC2_DIR);
CytronMD motor3(PWM_DIR, DC3_PWM, DC3_DIR);
CytronMD motor[3] = {motor1, motor2, motor3};

// Servo Calibration Arrays
int servo_off[4] = {100, 97, 90, 92};
int servo_val[4] = {0, 0, 0, 0};

// ----------------------
// ROS Callbacks
// ----------------------

// Callback for joint state updates
void joint_state_callback(const void *msgin) {
    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

    if (msg->position.size < 7 || msg->position.data == NULL) {
        publish_debug_message("[WARN] Invalid joint state message.");
        return;
    }

    float g0 = msg->position.data[6];
    float g1 = msg->position.data[5];
    float g2 = msg->position.data[4];
    float g3 = msg->position.data[3];

    g3 = map_gimbal_to_servo(g3, G3_MIN, G3_MAX, -180, 180);
    g2 = map_gimbal_to_servo(g2, G2_MIN, G2_MAX, -90, 90);
    g1 = map_gimbal_to_servo(g1, G1_MIN, G1_MAX, -90, 90);
    g0 = map_gimbal_to_servo(g0, G0_MIN, G0_MAX, -90, 90);

    servo_val[0] = -g0 / 2 + g1 + servo_off[0];
    servo_val[1] = g0 / 2 + g1 + servo_off[1];
    servo_val[2] = g2 + servo_off[2];
    servo_val[3] = g3 + servo_off[3];

    servo1.write(servo_val[0]);
    servo2.write(servo_val[1]);
    servo3.write(servo_val[2]);
    servo4.write(servo_val[3]);
}

// Callback for target pose updates
void target_pose_callback(const void *msgin) {
    const geometry_msgs__msg__PoseStamped *msg = (const geometry_msgs__msg__PoseStamped *)msgin;

    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    JointAngles desiredAngles = computePSMJointAngles(x, y, z);

    char debugBuf[128];
    snprintf(debugBuf, sizeof(debugBuf), "Desired Angles -> q1: %.2f, q2: %.2f, q3: %.2f",
             desiredAngles.q1, desiredAngles.q2, desiredAngles.q3);
    publish_debug_message(debugBuf);

    PIDupdate(&desiredAngles.q1, 0, "PID", 20.0f, 50.0f, 0.50f);
    PIDupdate(&desiredAngles.q2, 1, "PI", 60.0f, 110.0f, 0.50f);
}

// ----------------------
// Setup Function
// ----------------------
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    // Setup limit switches
    pinMode(LS1_NO, INPUT_PULLUP);
    pinMode(LS2_NO, INPUT_PULLUP);
    pinMode(LS3_NO, INPUT_PULLUP);
    pinMode(LS1_NC, INPUT_PULLUP);
    pinMode(LS2_NC, INPUT_PULLUP);
    pinMode(LS3_NC, INPUT_PULLUP);

    // Attach servos and set initial positions
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
    servo1.write(servo_off[0]);
    servo2.write(servo_off[1]);
    servo3.write(servo_off[2]);
    servo4.write(servo_off[3]);

    // Set motor PWM frequency
    float PWM_freq = 18500.0;
    analogWriteFrequency(DC1_PWM, PWM_freq);
    analogWriteFrequency(DC2_PWM, PWM_freq);
    analogWriteFrequency(DC3_PWM, PWM_freq);

    // ROS 2 setup
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "psm_sensor_node", "", &support));

    // Subscriptions
    RCCHECK(rclc_subscription_init_default(
        &joint_state_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/mtm_joint_states"));

    RCCHECK(rclc_subscription_init_default(
        &target_pose_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        "/target_pose"));

    // Publishers
    RCCHECK(rclc_publisher_init_default(
        &sensor_data_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/psm_sensor_data"));

    RCCHECK(rclc_publisher_init_default(
        &debug_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/psm_debug"));

    // Allocate and initialize message memory
    sensor_data_msg.data.data = (int32_t *)malloc(3 * sizeof(int32_t));
    sensor_data_msg.data.size = 3;
    sensor_data_msg.data.capacity = 3;

    debug_msg.data.data = (char *)malloc(128 * sizeof(char));
    debug_msg.data.size = 0;
    debug_msg.data.capacity = 128;

    target_pose_msg.pose.position.x = 0;
    target_pose_msg.pose.position.y = 0;
    target_pose_msg.pose.position.z = 0;

    memset(&received_joint_state, 0, sizeof(sensor_msgs__msg__JointState));
    received_joint_state.name.data = (rosidl_runtime_c__String *)malloc(JOINT_NAME_MAX * sizeof(rosidl_runtime_c__String));
    received_joint_state.name.size = JOINT_NAME_MAX;
    received_joint_state.name.capacity = JOINT_NAME_MAX;
    for (size_t i = 0; i < received_joint_state.name.size; i++) {
        rosidl_runtime_c__String__init(&received_joint_state.name.data[i]);
        received_joint_state.name.data[i].data = (char *)malloc(NAME_LENGTH_MAX);
        received_joint_state.name.data[i].size = 0;
        received_joint_state.name.data[i].capacity = NAME_LENGTH_MAX;
    }
    received_joint_state.position.data = (double *)malloc(JOINT_NAME_MAX * sizeof(double));
    received_joint_state.position.size = JOINT_NAME_MAX;
    received_joint_state.position.capacity = JOINT_NAME_MAX;

    // Executor setup
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &joint_state_subscriber, &received_joint_state, &joint_state_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &target_pose_subscriber, &target_pose_msg, &target_pose_callback, ON_NEW_DATA));

    home_motors();
}

// ----------------------
// Loop Function
// ----------------------
void loop() {
    // 1. Read encoder values and publish sensor data
    read_encoder_data(&sensor_data_msg);
    RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));

    // 2. Process incoming ROS messages
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));

    // 3. Compute desired joint angles from target pose
    double x_target = target_pose_msg.pose.position.x;
    double y_target = target_pose_msg.pose.position.y;
    double z_target = target_pose_msg.pose.position.z;
    JointAngles desiredAngles = computePSMJointAngles(x_target, y_target, z_target);

    // 4. Publish debug message with computed angles
    char debugBuf[128];
    snprintf(debugBuf, sizeof(debugBuf), "Desired Angles -> q1: %.2f, q2: %.2f, q3: %.2f",
             desiredAngles.q1, desiredAngles.q2, desiredAngles.q3);
    publish_debug_message(debugBuf);

    // 5. Drive joints with PID controller
    PIDupdate(&desiredAngles.q1, 0, "PID", 20.0f, 50.0f, 0.50f);
    PIDupdate(&desiredAngles.q2, 1, "PI", 60.0f, 110.0f, 0.50f);

    delay(10);
}
