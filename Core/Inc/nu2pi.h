#ifndef NU2PI_H_
#define NU2PI_H_

#include <stdint.h>

struct nu2pi{
    // a_delta, b_delta, c_delta represent encoder tick change.
    int8_t a_delta;
    int8_t b_delta;
    int8_t c_delta;
    // IMU Readings
    float gyro[3];
    float accel[3];
    float mag[3];
    // Motor current
    float curr[3];
    // Battery
    int8_t batt;
    // Error Code
    int8_t error;
};

#endif
