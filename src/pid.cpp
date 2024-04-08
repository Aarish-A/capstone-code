#include "pid.h"

// Global Extern Variables
PIDController ratioPID;
PIDController motorPID;

// Initialize the PID controller parameters
void initPID(PIDController *pid, double kp, double ki, double kd, double setpoint, double outputMin, double outputMax, int sampleTimeMs) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0;
    pid->error = 0.0;
    pid->prevError = 0.0;
    pid->outputMin = outputMin;
    pid->outputMax = outputMax;
    pid->sampleTimeMs = sampleTimeMs;
}

// Update the PID controller
double updatePID(PIDController *pid, double currentValue) {
    double deltaTime = (double)pid->sampleTimeMs / 1000.0; // Convert milliseconds to seconds

    pid->error = pid->setpoint - currentValue;
    pid->integral += pid->error * deltaTime;
    double derivative = (pid->error - pid->prevError) / deltaTime;
    double output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;

    // Clamp output to min/max
    if (output > pid->outputMax)
        output = pid->outputMax;
    if (output < pid->outputMin)
        output = pid->outputMin;

    pid->prevError = pid->error;

    return output;
}

double updatePIDCascading(PIDController *pidOuter, double pidOuterCurrentValue, PIDController *pidInner, double pidInnerCurrentValue) {
    double deltaTime = (0.005);  // Time step
    double outputOuter, outputInner;

    outputOuter = updatePID(pidOuter, pidOuterCurrentValue);
    pidInner->setpoint = outputOuter;
    // TODO: Convert ratio to encoder position
    outputInner = updatePID(pidInner, pidInnerCurrentValue);

    return outputInner;
}