#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdbool.h>

#include "config.h"
#include "sensors.h"

void initMotor();              // Initialize motor
void setMotor(int dutyCycle);  // Set motor given a duty cycle

// Global extern variables
extern int dutyCycle;  // Duty Cycle to pass to motor