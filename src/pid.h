#pragma once

// Structure to hold PID controller parameters
typedef struct {
    double kp;         // Proportional gain
    double ki;         // Integral gain
    double kd;         // Derivative gain
    double setpoint;   // Desired setpoint
    double integral;   // Integral term accumulator
    double prevError;  // Previous error, for derivative term
    double outputMin;  // Minimum output value
    double outputMax;  // Maximum output value
} PIDController;

void initPID(PIDController *pid, double kp, double ki, double kd, double setpoint, double outputMin, double outputMax);
double updatePID(PIDController *pid, double currentValue, double deltaTime);
double updatePIDCascading(PIDController *pidOuter, double pidOuterCurrentValue, PIDController *pidInner, double pidInnerCurrentValue);
