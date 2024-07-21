#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PID {
public:
    PID(float Kp, float Ki, float Kd, float dt);
    float bucle_control(float setpoint, float measured_value);

private:
    float Kp, Ki, Kd, dt;
    float previous_error, integral;
};

class PID_IIR {
public:
    PID_IIR(float Kp, float Ki, float Kd, float dt);
    float bucle_control(float setpoint, float measured_value);

private:
    float A0, A1, A2;
    float error[3];
    float output;
};

class PID_LowPass {
public:
    PID_LowPass(float Kp, float Ki, float Kd, float dt);
    float bucle_control(float setpoint, float measured_value);

private:
    const int N = 5;
    float A0, A1;
    float A0d, A1d, A2d;
    float tau, alpha, alpha_1, alpha_2;
    float error[3];
    float d0, d1, fd0, fd1;
    float output;
};

#endif
