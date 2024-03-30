#pragma once

// Structure to hold PID controller parameters
typedef struct {
    double kp;         // Proportional gain
    double ki;         // Integral gain
    double kd;         // Derivative gain
    double setpoint;   // Desired setpoint
    double integral;   // Integral term accumulator
    double error;      // Current error
    double prevError;  // Previous error, for derivative term
    double outputMin;  // Minimum output value
    double outputMax;  // Maximum output value
} PIDController;

void initPID(PIDController *pid, double kp, double ki, double kd, double setpoint, double outputMin, double outputMax);                 // Initialize PID controller
double updatePID(PIDController *pid, double currentValue, double deltaTime);                                                            // Update PID controller
double updatePIDCascading(PIDController *pidOuter, double pidOuterCurrentValue, PIDController *pidInner, double pidInnerCurrentValue);  // Update two PID controllers with cascading logic

// Global Extern Variables
extern PIDController ratioPID;
extern PIDController motorPID;
