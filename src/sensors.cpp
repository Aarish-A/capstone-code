#include "sensors.h"

AS5600 Encoder;
SensorValues sensors;
int throttleMin = 4095;
int throttleMax = 0;

void initSensors() {
    pinMode(PIN_BRAKE_SENSOR, INPUT);

    // Set the pin modes for the sensor inputs
    pinMode(PIN_ENGINE_RPM_SENSOR, INPUT);
    pinMode(PIN_BATTERY_SENSOR, INPUT);

    // Set the pin modes for the brake input and launch button
    pinMode(PIN_BRAKE_SENSOR, INPUT);
    pinMode(PIN_LAUNCH_BUTTON, INPUT_PULLUP);

    Encoder.begin();
}

void updateSensors() {
    // Throttle comes from serial reading
    // Engine RPM comes from serial reading
    sensors.helix = readHelix();
    sensors.brake = digitalRead(PIN_BRAKE_SENSOR);
}

float readHelix() {
    float rawHelix = (Encoder.readAngle() * AS5600_RAW_TO_DEGREES);
    return rawHelix;  //- helixOffset;
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