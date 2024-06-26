#pragma once

#include <BluetoothSerial.h>

#include "sensors.h"

// Global Extern Variables
extern BluetoothSerial SerialBT;
extern String incomingBluetoothData;

void initBluetooth();                    // Initialize bluetooth system
void readBluetoothData();                // Read bluetooth data
void processBluetoothData(String data);  // Process incoming bluetooth data

// Base case for a single argument pair
template <typename T, typename V>
void logBluetooth(T label, V value) {
    if (!SerialBT.connected())
        return;

    SerialBT.print(label);
    SerialBT.print(": ");
    SerialBT.println(value);
}

// Template function to handle pairs of arguments
template <typename T, typename V, typename... Args>
void logBluetooth(T label, V value, Args... args) {
    if (!SerialBT.connected())
        return;

    SerialBT.print(label);
    SerialBT.print(": ");
    SerialBT.print(value);
    SerialBT.print(", ");   // Separate pairs with a comma
    logBluetooth(args...);  // Recursive call with the remaining arguments
}