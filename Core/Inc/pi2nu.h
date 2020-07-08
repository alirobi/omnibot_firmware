//#include<stdio.h>
//#include<stdlib.h>

struct pi2nu{
    // Total 13 bytes
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
};

struct PID{
    // 13 bytes
    int8_t header;
    float pGain;
    float iGain;
    float dGain;
};


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
