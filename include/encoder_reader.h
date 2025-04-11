#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <sensor_msgs/msg/joint_state.h>
#include <Encoder.h>

// Declare encoder objects as extern
extern Encoder Enc1;
extern Encoder Enc2;
extern Encoder Enc3;

// Function to read encoder data
void read_encoder_data(sensor_msgs__msg__JointState *msg);

#endif // ENCODER_READER_H