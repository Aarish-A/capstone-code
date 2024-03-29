#include "pid.h"

// Initialize the PID controller parameters
void initPID(PIDController *pid, double kp, double ki, double kd, double setpoint, double outputMin, double outputMax) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0;
    pid->prevError = 0.0;
    pid->outputMin = outputMin;
    pid->outputMax = outputMax;
}

// Update the PID controller
double updatePID(PIDController *pid, double currentValue, double deltaTime) {
    double error = pid->setpoint - currentValue;
    pid->integral += error * deltaTime;
    double derivative = (error - pid->prevError) / deltaTime;
    double output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // Clamp output to min/max
    if (output > pid->outputMax)
        output = pid->outputMax;
    if (output < pid->outputMin)
        output = pid->outputMin;

    pid->prevError = error;

    return output;
}

double updatePIDCascading(PIDController *pidOuter, double pidOuterCurrentValue, PIDController *pidInner, double pidInnerCurrentValue) {
    double deltaTime = 0.01;  // Time step
    double outputOuter, outputInner;

    outputOuter = updatePID(pidOuter, pidOuterCurrentValue, deltaTime);
    pidInner->setpoint = outputOuter;
    // TODO: Convert ratio to encoder position
    outputInner = updatePID(pidInner, pidInnerCurrentValue, deltaTime);

    return outputInner;
}