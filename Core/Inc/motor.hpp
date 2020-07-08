class Motor{
private:
    // static int motorNum = 0;
    float pGain, iGain, dGain;
    float targetSpeed;
    float command;
public:
    float currentSpeed;

    Motor(float p, float i, float d);
    void setPID(float p, float i, float d);
    void move();    
    void getTarSpeed(float speed);
    void calcCurSpeed();
    void pid();
};

Motor::Motor(){}

