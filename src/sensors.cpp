#include "sensors.h"

AS5600 Encoder;

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

void updateSensors(SensorValues *sensors) {
    // Throttle comes from serial reading
    // Engine RPM comes from serial reading
    sensors->helix = readHelix();
    sensors->brake = digitalRead(PIN_BRAKE_SENSOR);
}

float readHelix() {
    float rawHelix = (Encoder.readAngle() * AS5600_RAW_TO_DEGREES);
    return rawHelix;  //- helixOffset;
}