#pragma once

#include <HardwareSerial.h>
#include <Wire.h>

#include "config.h"
#include "sensors.h"

void initSerial();                     // Initialize serial input
void readOnboardData();                // Read serial data
void processOnboardData(String data);  // Process serial data
void exportOnboardData();              // Export (write) back to serial output

// Global Extern Variables
extern HardwareSerial Onboard;
extern String incomingOnboardData;
