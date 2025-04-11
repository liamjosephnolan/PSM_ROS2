#ifndef CONFIG_H
#define CONFIG_H

// === Configuration ===
// Define Pins
#define ENC1_A 0
#define ENC1_B 1
#define ENC2_A 2
#define ENC2_B 3
#define ENC3_A 4
#define ENC3_B 5

#define DC1_PWM 6
#define DC2_PWM 7
#define DC3_PWM 8

#define DC1_DIR 24
#define DC2_DIR 25
#define DC3_DIR 26

#define LS1_NC 22
#define LS1_NO 23
#define LS2_NC 20
#define LS2_NO 21
#define LS3_NC 18
#define LS3_NO 19

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

#endif // CONFIG_H