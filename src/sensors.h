#pragma once

#include <AS5600.h>
#include <Arduino.h>
#include <stdbool.h>

#include "config.h"

// Structure for sensors
typedef struct {
    float throttle;
    float helix;
    int engineRPM;
    bool brake;
} SensorValues;

void initSensors();              // Initialize all sensors
void updateSensors();            // Update all sensors
float readHelix();               // Read helix posiiton
float potRead(int rawPotValue);  // Read raw poti value

// Global Extern Variables
extern SensorValues sensors;
extern AS5600 Encoder;
extern int throttleMin;
extern int throttleMax;
