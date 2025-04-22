#include "config.h"
#include <cstdio>  // For snprintf

// A helper function to constrain a value
double constrain_value(double value, double min_val, double max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

JointAngles computePSMJointAngles(double x_p, double y_p, double z_p) {
    // Apply the transformations
    double x = -x_p * 1000;      // Invert X
    double y = z_p * 1000;       // Swap Y and Z
    double z = y_p * 1000;
    
    // Format a debug message and publish it
    char debug_message[128];
    snprintf(debug_message, sizeof(debug_message), "x: %.2f, y: %.2f, z: %.2f", x, y, z);
    publish_debug_message(debug_message);

    JointAngles angles;
    // Compute insertion distance as double, then cast to float
    angles.q3 = static_cast<float>(std::sqrt(x * x + y * y + z * z));
    
    // Compute pitch angle in degrees and constrain
    double pitch = std::atan2(x, y) * 180.0 / M_PI;
    pitch = constrain_value(pitch, -10.0, 10.0);
    angles.q2 = static_cast<float>(pitch);

    // Compute yaw angle in degrees and constrain
    double yaw = std::atan2(z, y) * 180.0 / M_PI;
    yaw = constrain_value(yaw, -10.0, 10.0);
    angles.q1 = static_cast<float>(yaw);

    return angles;
}