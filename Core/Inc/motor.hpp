class Motor{
private:
    // static int motorNum = 0;
    float pGain, iGain, dGain;
    float samplingTime;
    float targetSpeed;
    float command;
    float error;
    float lastError;
    float iError;
    float dError;
    float cutoffFreq;
    float filtConst;
public:
    float currentSpeed;

    Motor(float p, float i, float d, float samTime);
    void setPID(float p, float i, float d);
    void move();    
    void setTarSpeed(float speed);
    void calcCurSpeed();
    void pid();
};

Motor::Motor(float p, float i, float d, float samTime, float cfFreq):pGain(p), iGain(i), dGain(d), samplingTime(samTime), cutoffFreq(cfFreq){
    filtConst = exp(-cutoffFreq*samplingTime);
}
void Motor::Motor(float p, float i, float d){
    pGain = p;
    iGain = i;
    dGain = d;
}
void move(){
    // Sent PWM;
}
void setTarSpeed(float speed){
    targetSpeed = speed;
}
void calcCurSpeed(){
    // Read Encoder;
}
void pid(){
    error = targetSpeed - currentSpeed;
    iError += (error + lastError) / 2 * samplingTime;
    dError = filtConst * ((error - lastError) / samplingTime) + (1-filtConst) * dError;
    command = pGain*error + iGain*iError + dGain*dError;
    lastError = error;
}
