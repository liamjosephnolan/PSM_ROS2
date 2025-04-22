#include <cmath>
#include "config.h"  // Keep your existing configuration


double constrain_value(double value, double min_val, double max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

void computePSMJointAngles(double x_p, double y_p, double z_p, 
        double& q1_p, double& q2_p, double& q3_p) {
    // Apply the transformations
    double x = -x_p*1000;      // Invert X
    double y = z_p*1000;       // Swap Y and Z
    double z = y_p*1000;
    publish_debug_message("x: %f, y: %f, z: %f", x, y, z);

    
    // Insertion distance (Euclidean distance)
    q3_p = std::sqrt(x * x + y * y + z * z);

    // Pitch angle in degrees (angle in XZ-plane) constrained to [-35, 35]
    q2_p = std::atan2(x, y) * 180.0 / M_PI;
    q2_p = constrain(q2_p, -10.0, 10.0); // These are currently over constrained to not break the robot normal are -35 35

    // Yaw angle in degrees (angle in XY-plane) constrained to [-30, 30]
    q1_p = std::atan2(z, y) * 180.0 / M_PI;
    q1_p = constrain(q1_p, -10.0, 10.0); // These are currently over constrained to not break the robot normal are -30 30
}