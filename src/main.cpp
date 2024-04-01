#include <Arduino.h>

#include "bluetooth.h"
#include "config.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "serial.h"
#include "state.h"
#include "utilities.h"

// Tasks that have predictable execution times should run periodically
// Else, tasks with unpredictable/varying execution time should run on a delay
#define TASK_PERIOD_SENSORS_MS 2
#define TASK_PERIOD_SERIAL_MS 5
#define TASK_DELAY_STATE_MS 5
#define TASK_DELAY_BLUETOOTH_MS 5

void TaskSensors(void* pvParameters);
void TaskSerial(void* pvParameters);
void TaskState(void* pvParameters);
void TaskBluetooth(void* pvParameters);

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
}

void TaskSensors(void* pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = TASK_PERIOD_SENSORS_MS / portTICK_PERIOD_MS;  // Convert ms to ticks

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // This task should execute every "TASK_PERIOD_SENSORS_MS"
    while (true) {
        updateSensors();

        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskSerial(void* pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = TASK_PERIOD_SERIAL_MS / portTICK_PERIOD_MS;  // Convert ms to ticks

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // This task should execute every "TASK_PERIOD_SERIAL_MS"
    while (true) {
        readOnboardData();
        exportOnboardData("RPM", sensors.engineRPM,
                          "Brake", sensors.brake,
                          "Throttle", sensors.throttle,
                          "Helix", sensors.helix);

        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskState(void* pvParameters) {
    while (true) {
        int setpoint;  // Setpoint variable for PID (target Engine RPM)

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

        updateState();

        // Wait "TASK_DELAY_STATE_MS" until next cycle
        vTaskDelay(TASK_DELAY_STATE_MS / portTICK_PERIOD_MS);
    }
}

void TaskBluetooth(void* pvParameters) {
    while (true) {
        readBluetoothData();
        exportBluetoothData("RPM", sensors.engineRPM,
                            "Throttle", sensors.throttle,
                            "Helix", sensors.helix,
                            "DutyCycle", dutyCycle);

        // Wait "TASK_DELAY_BLUETOOTH_MS" until next cycle
        vTaskDelay(TASK_DELAY_BLUETOOTH_MS / portTICK_PERIOD_MS);
    }
}
