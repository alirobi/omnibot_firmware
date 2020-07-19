#ifndef NU2PI_H_
#define NU2PI_H_

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

void data2nu2pi(struct nu2pi* msg, int8_t a_delta, int8_t b_delta, int8_t c_delta, float gyro[3], float accel[3], float mag[3], float curr[3], int8_t batt, int8_t error){
    msg->a_delta = a_delta;
    msg->b_delta = b_delta;
    msg->c_delta = c_delta;
    msg->gyro[0] = gyro[0];
    msg->gyro[1] = gyro[1];
    msg->gyro[2] = gyro[2];
    msg->accel[0] = accel[0];
    msg->accel[1] = accel[1];
    msg->accel[2] = accel[2];
    msg->mag[0] = mag[0];
    msg->mag[1] = mag[1];
    msg->mag[2] = mag[2];
    msg->curr[0] = curr[0];
    msg->curr[1] = curr[1];
    msg->curr[2] = curr[2];
    msg->batt = batt;
    msg->error = error;
    int8_t *temp = (int8_t*)msg;
    msg->checkSum = 0;
    for(int i = 0; i < 41; i ++){
        msg->checkSum += temp[i];
    }
}

#endif
