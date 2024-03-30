#include "sensors.h"

AS5600 Encoder;
SensorValues sensors;
int throttleMin = 2910;
int throttleMax = 1820;

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
    float rawHelix = map(Encoder.readAngle(), 4095, 0, 0, 4095);
    return ((rawHelix * AS5600_RAW_TO_DEGREES) - HELIX_OFFSET);  //- helixOffset;
}

float potRead(int rawPotValue) {
    float pot = map(rawPotValue, throttleMin, throttleMax, 0, 100);

    if (pot < 5)
        pot = 0;
    else if (pot > 100)
        pot = 100;

    return pot;
}