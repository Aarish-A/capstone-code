#include <Arduino.h>

#include "bluetooth.h"
#include "config.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "serial.h"
#include "state.h"
#include "utilities.h"

PIDController ratioPID;
PIDController motorPID;
ECVTState currentState;

float potRead(int rawPotValue);
int throttleToRPM(float throttle);

// Serial communication

void setup() {
    initSensors();
    initPID(&ratioPID, RATIO_PID_KP, RATIO_PID_KI, RATIO_PID_KD, RATIO_PID_INIT_SETPOINT, RATIO_PID_OUTPUT_MIN, RATIO_PID_OUTPUT_MAX);
    initPID(&motorPID, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_INIT_SETPOINT, MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    currentState = Idle;
}

void loop() {
    // Read board data
    updateSensors(&sensors);
    readOnboardData();
    readBluetoothData();  // Read bluetooth data

    int setpoint;
    int dutyCycle;

    switch (currentState) {
        case Idle:
            dutyCycle = 0;
            break;
        case Brake:
            // TODO: Impelement later
            break;
        case Drive:
            setpoint = throttleToRPM(sensors.throttle);
            dutyCycle = updatePIDCascading(&ratioPID, sensors.engineRPM, &motorPID, sensors.helix);
            break;
    }

    setMotor(dutyCycle);  // Handles MOSFET switching (direction + duty cycle)

    currentState = updateState(currentState, &sensors);
    delay(5);
}

int throttleToRPM(float throttle) {
    if (throttle > 0 && throttle <= 0.1)
        return (int)mapFloat(throttle, 0.0, 0.1, 1500, 1600);
    else if (throttle > 0.1 && throttle <= 0.2)
        return (int)mapFloat(throttle, 0.1, 0.2, 1600, 1800);
    else if (throttle > 0.2 && throttle <= 0.3)
        return (int)mapFloat(throttle, 0.2, 0.3, 1800, 2000);
    else if (throttle > 0.3 && throttle <= 0.4)
        return (int)mapFloat(throttle, 0.3, 0.4, 2000, 2200);
    else if (throttle > 0.4 && throttle <= 0.5)
        return (int)mapFloat(throttle, 0.4, 0.5, 2200, 2500);
    else if (throttle > 0.5 && throttle <= 0.6)
        return (int)mapFloat(throttle, 0.5, 0.6, 2500, 2600);
    else if (throttle > 0.6 && throttle <= 0.7)
        return (int)mapFloat(throttle, 0.6, 0.7, 2600, 2700);
    else if (throttle > 0.7 && throttle <= 0.8)
        return (int)mapFloat(throttle, 0.7, 0.8, 2700, 2800);
    else if (throttle > 0.8 && throttle <= 0.9)
        return (int)mapFloat(throttle, 0.8, 0.9, 2800, 2900);
    else if (throttle > 0.9 && throttle <= 1)
        return (int)mapFloat(throttle, 0.9, 1.0, 2800, 3000);

    return 0;
}
