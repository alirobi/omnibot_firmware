#ifndef PI2NU_H_
#define PI2NU_H_

struct pi2nu{
    // Total 42 bytes
    // 8 bit integer to represent speed of each wheel
    // 0 for stop
    // + 127 max CCW
    // - 127 max CW
    int8_t header;
    int8_t vel_a;
    int8_t vel_b;
    int8_t vel_c;
    // Error code(State)
    int8_t error;
    int8_t dummy[8];
    int8_t checkSum;
};

void data2pi2nu(struct pi2nu* msg, int8_t vel_a, int8_t vel_b, int8_t vel_c, int8_t error){
    msg->header = 1;
    msg->vel_a = vel_a;
    msg->vel_b = vel_b;
    msg->vel_c = vel_c;
    msg->error = error;
    for(int i = 0; i<8; i++){
        msg->dummy[i] = 0;
    }
    msg->checkSum = vel_a + vel_b + vel_c + error;
}

struct PID{
    // 42 bytes
    int8_t header;
    float pGain;
    float iGain;
    float dGain;
    int8_t checkSum;
};

void data2PID(struct PID* msg, float pGain, float iGain, float dGain){
    msg->header = 2;
    msg->pGain = pGain;
    msg->iGain = iGain;
    msg->dGain = dGain;
    int8_t* temp = (int8_t*)msg;
    msg->checkSum = 0;
    for(int i = 0; i < 13; i++)
        msg->checkSum += temp[i];
}

#endif

//int main(){
//    struct pi2nu *msg;
//    msg = (struct pi2nu*) malloc(sizeof(struct pi2nu));
//    msg->vel_a = 1;
//    msg->vel_b = 2;
//    msg->vel_c = 3;
//    
//    printf("%d %d %d \n", msg->vel_a, msg->vel_b, msg->vel_c);
//
//    printf("%d %d %d\n", *(int8_t*) msg, *(int8_t*) msg+1, *(int8_t*) msg+2);    
//
//
//    int32_t test;
//    test = 127;
//    test = test << 8;
//    test += 100;
//    test = test << 8;
//    test += 50;
//    msg = &test;
//    printf("%d %d %d %d\n", msg->vel_a, msg->vel_b, msg->vel_c, msg->error);
//
//    return 0;
//}
