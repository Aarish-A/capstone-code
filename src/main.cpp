#include <Arduino.h>

#include "bluetooth.h"
#include "config.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "serial.h"
#include "state.h"
#include "utilities.h"

void setup() {
    // Initialize communications
    initBluetooth();
    initSerial();

    // Initialize inputs and outputs
    initSensors();
    initMotor();

    // Initialize PID controllers and state
    initStateMachine();
    initPID(&ratioPID, RATIO_PID_KP, RATIO_PID_KI, RATIO_PID_KD, RATIO_PID_INIT_SETPOINT, RATIO_PID_OUTPUT_MIN, RATIO_PID_OUTPUT_MAX);
    initPID(&motorPID, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_INIT_SETPOINT, MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
}

void loop() {
    // Read comms data
    readOnboardData();
    readBluetoothData();

    // Update onboard sensors
    updateSensors();

    int setpoint;   // Setpoint variable for PID (target Engine RPM)
    int dutyCycle;  // Duty Cycle to pass to motor

    switch (currentState) {
        case Idle:
            dutyCycle = 0;  // There should be no control of motor in idle
            break;
        case Brake:
            // TODO: Impelement later
            break;
        case Drive:
            // Engine RPM setpoint is determined by throttle position
            setpoint = mapThrottleToRPM(sensors.throttle);
            // Duty cycle is determined by output of cascading PID controller (to minimize engine RPM error)
            dutyCycle = updatePIDCascading(&ratioPID, sensors.engineRPM, &motorPID, sensors.helix);
            break;
    }

    setMotor(dutyCycle);  // setMotor handles MOSFET switching (direction + duty cycle)

    // Export (write) comms data
    exportOnboardData();
    exportBluetoothData();

    // Update state to new state
    updateState();

    delay(5);  // 5ms because that is the rate at which serial data comes in
}
