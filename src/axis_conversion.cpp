#include "config.h"

// Define the global variable res_avago
float res_avago = 2000; 

float Ax1toAngle(long count) {
    float q;
    float trans = 20.000; // Gear reduction ratio
    q = ((float)count / (res_avago * trans) * 360.0) - 30; // Convert encoder count to angle in degrees
    return q;
}

float Ax2toAngle(long count) {
    float q;
    float trans = 13.333;
    q = ((-1)*((float)count / (res_avago * trans) * 360.0)) - 30; // Convert encoder count to angle in degrees
    return q;
}

float Ax3toAngle(long count) {
    float q;
    float D = 19.10;
    float ref = 360;
    q = (1) * PI * D * (float)count * res_avago / ref;
    return q;
}

int Ax1toCounts(float angle) {
    int q;
    float trans = 20.000;
    q = int(-1.0 * angle * trans / res_avago);
    return q;
}

int Ax2toCounts(float angle) {
    int q;
    float trans = 13.333;
    q = int(-1.0 * angle * trans / res_avago);
    return q;
}

int Ax3toCounts(float pos) {
    int q;
    float D = 19.10;
    float ref = 360;
    q = int(1.0 * (pos * ref) / (PI * D * res_avago));
    return q;
}