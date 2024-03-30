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
void exportBluetoothData(T label, V value) {
    SerialBT.print(label);
    SerialBT.print(value);
}

// Template function to handle pairs of arguments
template <typename T, typename V, typename... Args>
void exportBluetoothData(T label, V value, Args... args) {
    SerialBT.print(label);
    SerialBT.print(":");
    SerialBT.print(value);
    SerialBT.print(", ");          // Separate pairs with a comma
    exportBluetoothData(args...);  // Recursive call with the remaining arguments
}