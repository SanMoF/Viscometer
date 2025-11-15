#include "PID_CAYETANO.h"
#include "SimpleGPIO.h"
PID_CAYETANO::PID_CAYETANO()
{
    integral = 0.0f;
    prev_error = 0.0f;
}

void PID_CAYETANO::setup(float Gains[3], float dt_us)
{
    _Kp = Gains[0];
    _Ki = Gains[1];
    _Kd = Gains[2];
    _dt = dt_us / 1000000.0f; // Convert microseconds to seconds
    integral = 0.0f;           // Reset integral
    prev_error = 0.0f;         // Reset previous error
}

float PID_CAYETANO::computedU(float error)
{
    // safety dt
    if (_dt <= 0.0f) {
        printf("PID ERROR: _dt <= 0 (%.9f)\n", _dt);
        return 0.0f;
    }

    // Proportional
    float P = _Kp * error;

    // Derivative
    float D = _Kd * (error - prev_error) / _dt;

    // Trapezoidal integration increment
    float error_avg = (error + prev_error) * 0.5f;
    float integral_update = error_avg * _dt;

    // Tentative integral (do not commit yet)
    float tentative_integral = integral + integral_update;

    // tentative output using tentative_integral
    float u_tentative = P + _Ki * tentative_integral + D;

    // Anti-windup policy: integrate only if tentative output is inside limits
    const float U_LIMIT = 100.0f;
    if (u_tentative > -U_LIMIT && u_tentative < U_LIMIT) {
        integral = tentative_integral; // commit integration
    } else {
        // don't integrate; optionally could slowly unwind integral
        // integral *= 0.999f; // small leak if desired
    }

    // clamp integral to reasonable bounds
    const float INTEGRAL_LIMIT = 100.0f;
    if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
    if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

    // final output
    float u = P + _Ki * integral + D;

    // clamp output
    if (u > U_LIMIT) u = U_LIMIT;
    if (u < -U_LIMIT) u = -U_LIMIT;

    // update previous error
    prev_error = error;

    // debug print (comment later)
    // printf("PID dbg: e=%.4f P=%.4f I=%.4f(Dint) D=%.4f u_tent=%.4f u=%.4f\n",error, P, _Ki*integral, D, u_tentative, u);

    return u;
}


void PID_CAYETANO::reset()
{
    integral = 0.0f;
    prev_error = 0.0f;
}