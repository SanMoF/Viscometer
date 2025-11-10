#ifndef __PID_CAYETANO_H__
#define __PID_CAYETANO_H__

class PID_CAYETANO
{
private:
    float _Kp;
    float _Ki;
    float _Kd;
    float _dt;
    float prev_error = 0.0f;
    float integral = 0.0f;

public:
    PID_CAYETANO();
    void setup(float Gains[3], float dt);
    float computedU(float error);
    void reset();
};

#endif // __PID_CAYETANO_H__