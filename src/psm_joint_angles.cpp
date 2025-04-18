#include <cmath>
#include "config.h"

void computePSMJointAngles(double x_p, double y_p, double z_p, double& q1_p, double& q2_p, double& q3_p) {
    const double d0 = -23.49; // Distance constant

    int sign = (y_p < 0) ? 1 : -1;

    // Compute q3 (insertion)
    q3_p = sign * std::sqrt(x_p * x_p + y_p * y_p + z_p * z_p) - d0;

    // Compute q2 (pitch)
    double s2 = -x_p / (d0 + q3_p);
    double c2 = std::sqrt(1 - s2 * s2);
    q2_p = std::atan2(s2, c2);
    if ((q2_p <= (-PI / 2)) || (q2_p >= (PI / 2))) {
        q2_p = std::atan2(s2, -c2);
    }
    q2_p *= (180.0 / PI);  // Convert to degrees

    // Compute q1 (yaw)
    double b = c2 * (q3_p + d0);
    double a = -b;
    double c = y_p + z_p;
    q1_p = std::atan2(b, a) + std::atan2(std::sqrt(a * a + b * b - c * c), c);
    if ((q1_p <= (-PI / 2)) || (q1_p >= (PI / 2))) {
        q1_p = std::atan2(b, a) - std::atan2(std::sqrt(a * a + b * b - c * c), c);
    }
    q1_p *= (180.0 / PI);  // Convert to degrees
}