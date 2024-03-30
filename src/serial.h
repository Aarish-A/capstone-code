#pragma once

#include <HardwareSerial.h>
#include <Wire.h>

#include "config.h"
#include "sensors.h"

void initSerial();
void readOnboardData();
void processOnboardData(String data);
void exportOnboardData();

// Global Extern Variables
extern HardwareSerial Onboard;
extern String incomingOnboardData;
