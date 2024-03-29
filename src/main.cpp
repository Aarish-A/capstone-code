#include <Arduino.h>
#include <BluetoothSerial.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "config.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "state.h"

SensorValues sensors;
PIDController ratioPID;
PIDController motorPID;
ECVTState currentState;

HardwareSerial Onboard(0);
BluetoothSerial SerialBT;  // Bluetooth debug

void readOnboardData();
void processOnboardData(String data);
void exportOnboardData();
void readBluetoothData();
void processBluetoothData(String data);
void exportBluetoothData();
float potRead(int rawPotValue);
int throttleToRPM(float throttle);

int throttleMin = 4095;
int throttleMax = 0;

// Serial communication
String incomingOnboardData = "";
String incomingBluetoothData = "";

void setup() {
    Onboard.begin(115200, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
    Wire.begin();
    SerialBT.begin("BajaECVT");

    initSensors();
    initPID(&ratioPID, RATIO_PID_KP, RATIO_PID_KI, RATIO_PID_KD, RATIO_PID_INIT_SETPOINT, RATIO_PID_OUTPUT_MIN, RATIO_PID_OUTPUT_MAX);
    initPID(&motorPID, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_INIT_SETPOINT, 0, 1);
    currentState = Idle;
}

void loop() {
    // Read board data
    updateSensors(&sensors);
    readOnboardData();
    if (SerialBT.connected()) {
        readBluetoothData();  // Read bluetooth data
    }

    int setpoint;
    float dutyCycle;

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

    setMotor(dutyCycle, &sensors);  // Handles MOSFET switching (direction + duty cycle)

    currentState = updateState(currentState, &sensors);
    delay(5);
}

float potRead(int rawPotValue) {
    if (rawPotValue < throttleMin) {
        throttleMin = rawPotValue;
    } else if (rawPotValue > throttleMax) {
        throttleMax = rawPotValue;
    }

    float pot = (map(rawPotValue, throttleMin, throttleMax, 0, 1000)) / 1000.0;

    if (pot < 0) {
        pot = 0;
    } else if (pot > 100) {
        pot = 100;
    }

    return pot;
}

float mapValue(float value, float srcRangeMin, float srcRangeMax, float targetRangeMin, float targetRangeMax) {
    // Ensure the input is within the source range
    value = (value < srcRangeMin) ? srcRangeMin : value;
    value = (value > srcRangeMax) ? srcRangeMax : value;

    // Calculate how 'far' the value is in the source range (0.0 to 1.0)
    float srcRangeScale = (value - srcRangeMin) / (srcRangeMax - srcRangeMin);

    // Map the 'far' value to the target range
    float mappedValue = targetRangeMin + (srcRangeScale * (targetRangeMax - targetRangeMin));

    return mappedValue;
}

int throttleToRPM(float throttle) {
    if (throttle > 0 && throttle <= 0.1)
        return (int)mapValue(throttle, 0.0, 0.1, 1500, 1600);
    else if (throttle > 0.1 && throttle <= 0.2)
        return (int)mapValue(throttle, 0.1, 0.2, 1600, 1800);
    else if (throttle > 0.2 && throttle <= 0.3)
        return (int)mapValue(throttle, 0.2, 0.3, 1800, 2000);
    else if (throttle > 0.3 && throttle <= 0.4)
        return (int)mapValue(throttle, 0.3, 0.4, 2000, 2200);
    else if (throttle > 0.4 && throttle <= 0.5)
        return (int)mapValue(throttle, 0.4, 0.5, 2200, 2500);
    else if (throttle > 0.5 && throttle <= 0.6)
        return (int)mapValue(throttle, 0.5, 0.6, 2500, 2600);
    else if (throttle > 0.6 && throttle <= 0.7)
        return (int)mapValue(throttle, 0.6, 0.7, 2600, 2700);
    else if (throttle > 0.7 && throttle <= 0.8)
        return (int)mapValue(throttle, 0.7, 0.8, 2700, 2800);
    else if (throttle > 0.8 && throttle <= 0.9)
        return (int)mapValue(throttle, 0.8, 0.9, 2800, 2900);
    else if (throttle > 0.9 && throttle <= 1)
        return (int)mapValue(throttle, 0.9, 1.0, 2800, 3000);
}

// Function to read onboard data from UART
void readOnboardData() {
    while (Onboard.available() > 0) {
        char c = Onboard.read();

        if (c == ',') {
            processOnboardData(incomingOnboardData);
            incomingOnboardData = "";
        } else if (c != '\n') {
            incomingOnboardData += c;
        }
    }
}

void readBluetoothData() {
    while (SerialBT.available() > 0) {
        char c = SerialBT.read();

        if (c == ',') {
            processBluetoothData(incomingBluetoothData);
            incomingBluetoothData = "";
        } else if (c != '\n') {
            incomingBluetoothData += c;
        }
    }
}

// Function to process onboard data received from UART
void processOnboardData(String data) {
    int dataIndex = data.indexOf(':');

    if (dataIndex != -1) {
        String item = data.substring(0, dataIndex);

        if (item == "RPM") {
            sensors.engineRPM = data.substring(dataIndex + 1).toInt();
        } else if (item == "Throttle") {
            sensors.throttle = potRead(data.substring(dataIndex + 1).toInt());
        } else if (item == "Launch") {
            // launchActive = data.substring(dataIndex + 1).toInt();
        }
    }
}

void processBluetoothData(String data) {
    int dataIndex = data.indexOf(':');

    if (dataIndex != -1) {
        String item = data.substring(0, dataIndex);

        if (item == "RPM") {
            sensors.engineRPM = data.substring(dataIndex + 1).toInt();
        } else if (item == "Throttle") {
            sensors.throttle = (data.substring(dataIndex + 1).toInt()) / 4095.0;
        } else if (item == "Launch") {
            // launchActive = data.substring(dataIndex + 1).toInt();
        } else if (item == "Helix") {
            sensors.helix = data.substring(dataIndex + 1).toInt();
        }
    }
}

// Function to export onboard data to UART
void exportOnboardData() {
    // Onboard.print("Battery:");  // 0-1
    // Onboard.print(batPercent);
    // Onboard.print(",");

    Onboard.print("RPM:");  // 0-1
    Onboard.print(sensors.engineRPM);
    Onboard.print(",");

    // Onboard.print("Launch:");  // 0-1
    // Onboard.print(launchActive);
    // Onboard.print(",");

    Onboard.print("Brake:");  // 0-1
    Onboard.print(sensors.brake);
    Onboard.print(",");

    Onboard.print("Throttle:");  // 0-1
    Onboard.print(sensors.throttle);
    Onboard.print(",");

    Onboard.print("Helix:");  // 0-1
    Onboard.print(sensors.helix);
    Onboard.println(",");
}

void exportBluetoothData() {
    // SerialBT.print("Battery:");  // 0-1
    // SerialBT.print(batPercent);
    // SerialBT.print(",");

    SerialBT.print("RPM:");  // 0-1
    SerialBT.print(sensors.engineRPM);
    SerialBT.print(",");

    // SerialBT.print("Launch:");  // 0-1
    // SerialBT.print(launchActive);
    // SerialBT.print(",");

    SerialBT.print("Brake:");  // 0-1
    SerialBT.print(sensors.brake);
    SerialBT.print(",");

    SerialBT.print("Throttle:");  // 0-1
    SerialBT.print(sensors.throttle);
    SerialBT.print(",");

    SerialBT.print("Helix:");  // 0-1
    SerialBT.print(sensors.helix);
    SerialBT.println(",");
}