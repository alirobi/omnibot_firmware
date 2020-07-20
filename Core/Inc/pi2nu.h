#ifndef PI2NU_H_
#define PI2NU_H_

#include <stdint.h>

struct pi2nu{
    // 8 bit integer to represent speed of each wheel
    // 0 for stop
    // + 127 max CCW
    // - 127 max CW
    int8_t vel_a;
    int8_t vel_b;
    int8_t vel_c;
    // Error code(State)
    int8_t error;
};

struct PID{
    float pGain;
    float iGain;
    float dGain;
};

#endif
