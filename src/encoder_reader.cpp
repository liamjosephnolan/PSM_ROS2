#include "encoder_reader.h"
#include "config.h" // Include shared pin definitions

// Define encoder objects
Encoder Enc1(ENC1_B, ENC1_A);
Encoder Enc2(ENC2_B, ENC2_A);
Encoder Enc3(ENC3_B, ENC3_A);

void read_encoder_data(sensor_msgs__msg__JointState *msg) {
  // Read encoder values
  msg->position.data[0] = Enc1.read();
  msg->position.data[1] = Enc2.read();
  msg->position.data[2] = Enc3.read();

  // Ensure the size matches the number of encoders
  msg->position.size = 3;
}