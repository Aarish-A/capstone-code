#pragma once

#include <AS5600.h>
#include <Arduino.h>
#include <stdbool.h>

#include "config.h"

typedef struct {
    float throttle;
    float helix;
    int engineRPM;
    bool brake;
} SensorValues;

extern AS5600 Encoder;

void initSensors();
void updateSensors(SensorValues *sensors);

float readThrottle();
float readHelix();

// Keep track of for sensors:
//  - Previous sensor value
//  - Current sensor value
//  - How long it's been seen sensor value switched
//  - Sensor value sensitivity