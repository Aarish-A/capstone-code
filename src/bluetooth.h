#pragma once

#include <BluetoothSerial.h>

#include "sensors.h"

void initBluetooth();                    // Initialize bluetooth system
void readBluetoothData();                // Read bluetooth data
void processBluetoothData(String data);  // Process incoming bluetooth data
void exportBluetoothData();              // Export (write) bluetooth data

// Global Extern Variables
extern BluetoothSerial SerialBT;
extern String incomingBluetoothData;