struct nu2pi{
    // Total 42 bytes
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

    int8_t checkSum;
};
