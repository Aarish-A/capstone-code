#pragma once

#include <HardwareSerial.h>
#include <Wire.h>

#include "config.h"
#include "sensors.h"

// Global Extern Variables
extern HardwareSerial Onboard;
extern String incomingOnboardData;

void initSerial();                     // Initialize serial input
void readOnboardData();                // Read serial data
void processOnboardData(String data);  // Process serial data

// Base case for a single argument pair
template <typename T, typename V>
void exportOnboardData(T label, V value) {
    Onboard.print(label);
    Onboard.print(": ");
    Onboard.println(value);
}

// Template function to handle pairs of arguments
template <typename T, typename V, typename... Args>
void exportOnboardData(T label, V value, Args... args) {
    Onboard.print(label);
    Onboard.print(": ");
    Onboard.print(value);
    Onboard.print(", ");         // Separate pairs with a comma
    exportOnboardData(args...);  // Recursive call with the remaining arguments
}