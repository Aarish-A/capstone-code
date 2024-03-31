#include <Arduino.h>

#include "bluetooth.h"
#include "config.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "serial.h"
#include "state.h"
#include "utilities.h"

// Comment out defines to remove that part of code
// #define TEST_SENSORS
// #define TEST_MOTOR
// #define TEST_SERIAL
// #define TEST_BLUETOOTH
// #define TEST_STATE

void setup() {
// Initialize communications
#ifdef TEST_BLUETOOTH
    initBluetooth();
#endif

#ifdef TEST_SERIAL
    initSerial();
#endif

    // Initialize inputs and outputs
#ifdef TEST_SENSORS
    initSensors();
#endif

#ifdef TEST_MOTOR
    initMotor();
#endif

    // Initialize PID controllers and state
#ifdef TEST_STATE
    initStateMachine();
    initPID(&ratioPID, RATIO_PID_KP, RATIO_PID_KI, RATIO_PID_KD, RATIO_PID_INIT_SETPOINT, RATIO_PID_OUTPUT_MIN, RATIO_PID_OUTPUT_MAX);
    initPID(&motorPID, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_INIT_SETPOINT, MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
#endif
}

void loop() {
// Read comms data
#ifdef TEST_BLUETOOTH
    readBluetoothData();
#endif

#ifdef TEST_SERIAL
    readOnboardData();
#endif

    // Update onboard sensors
#ifdef TEST_SENSORS
    updateSensors();
#endif

    int dutyCycle = 0;  // Duty Cycle to pass to motor
    int setpoint = 0;   // Setpoint variable for PID (target Engine RPM)

#ifdef TEST_STATE

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
#endif

#ifdef TEST_MOTOR
    setMotor(dutyCycle);  // setMotor handles MOSFET switching (direction + duty cycle)
#endif

#ifdef TEST_SERIAL
    // Export (write) comms data
    exportOnboardData("RPM", sensors.engineRPM,
                      "Brake", sensors.brake,
                      "Throttle", sensors.throttle,
                      "Helix", sensors.helix);
#endif

#ifdef TEST_BLUETOOTH
    exportBluetoothData("RPM", sensors.engineRPM,
                        "Throttle", sensors.throttle,
                        "Helix", sensors.helix,
                        "DutyCycle", dutyCycle);
#endif

#ifdef TEST_STATE
    // Update state to new state
    updateState();
#endif

    delay(5);  // 5ms because that is the rate at which serial data comes in
}
