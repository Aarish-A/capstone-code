#pragma once

// Low Power Input Pin Configuration
#define PIN_ENGINE_RPM_SENSOR 12
#define PIN_LAUNCH_BUTTON 15  // can change this one
#define PIN_BRAKE_SENSOR 27
#define PIN_THROTTLE_SENSOR 35  // Not confirmed
#define PIN_BATTERY_SENSOR 36
#define PIN_HELIX_SENSOR 37  // Not confirmed

#define PIN_LINK_RX 3
#define PIN_LINK_TX 1

// High Power Output Pin Configuration
#define PIN_MOTOR_FORWARD_A 32  // Forward MOSFET (+)
#define PIN_MOTOR_FORWARD_B 26  // Forward MOSFET (-)
#define PIN_MOTOR_REVERSE_A 25  // Reverse MOSFET (+)
#define PIN_MOTOR_REVERSE_B 33  // Reverse MOSFET (-)

// Electrical System Constants
#define PWM_RESOLUTION 256

// Physical System Constants
#define BELT_LENGTH 1
#define HELIX_MIN_ANGLE 0   // Degrees
#define HELIX_MAX_ANGLE 47  // Degrees
#define HELIX_OFFSET 166    // Degrees

// Ratio PID Constants
#define RATIO_PID_KP 1
#define RATIO_PID_KI 0.01
#define RATIO_PID_KD 0.1
#define RATIO_PID_OUTPUT_MIN 0.68
#define RATIO_PID_OUTPUT_MAX 3.41
#define RATIO_PID_INIT_SETPOINT RATIO_PID_OUTPUT_MIN

// Motor PID Constants
#define MOTOR_PID_KP 1
#define MOTOR_PID_KI 0.01
#define MOTOR_PID_KD 0.1
#define MOTOR_PID_OUTPUT_MIN -(PWM_RESOLUTION - 1)
#define MOTOR_PID_OUTPUT_MAX +(PWM_RESOLUTION - 1)
#define MOTOR_PID_INIT_SETPOINT 0

#define SAMPLE_TIME_MS 5
