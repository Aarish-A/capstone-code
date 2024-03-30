#pragma once

#include <BluetoothSerial.h>

#include "sensors.h"

void initBluetooth();
void readBluetoothData();
void processBluetoothData(String data);
void exportBluetoothData();

// Global Extern Variables
extern BluetoothSerial SerialBT;  // Bluetooth debug
extern String incomingBluetoothData;