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

void initSensors();
void updateSensors(SensorValues *sensors);

float readThrottle();
float readHelix();
float potRead(int rawPotValue);

// Global Extern Variables
extern SensorValues sensors;
extern AS5600 Encoder;
extern int throttleMin;
extern int throttleMax;

// Keep track of for sensors:
//  - Previous sensor value
//  - Current sensor value
//  - How long it's been seen sensor value switched
//  - Sensor value sensitivity