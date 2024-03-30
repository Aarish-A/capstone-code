#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdbool.h>

#include "config.h"
#include "sensors.h"

void initMotor();
void setMotor(int dutyCycle, SensorValues *sensors);