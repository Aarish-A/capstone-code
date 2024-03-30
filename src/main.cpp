#include <Arduino.h>

#include "bluetooth.h"
#include "config.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "serial.h"
#include "state.h"
#include "utilities.h"

ECVTState currentState;

void setup() {
    initSensors();
    initPID(&ratioPID, RATIO_PID_KP, RATIO_PID_KI, RATIO_PID_KD, RATIO_PID_INIT_SETPOINT, RATIO_PID_OUTPUT_MIN, RATIO_PID_OUTPUT_MAX);
    initPID(&motorPID, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_INIT_SETPOINT, MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    currentState = Idle;
}

void loop() {
    // Read board data
    updateSensors();
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
            setpoint = mapThrottleToRPM(sensors.throttle);
            dutyCycle = updatePIDCascading(&ratioPID, sensors.engineRPM, &motorPID, sensors.helix);
            break;
    }

    setMotor(dutyCycle);  // Handles MOSFET switching (direction + duty cycle)

    exportOnboardData();
    exportBluetoothData();

    currentState = updateState(currentState);
    delay(5);
}
