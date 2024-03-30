#include "bluetooth.h"

// Global Extern Variables
BluetoothSerial SerialBT;  // Bluetooth debug
String incomingBluetoothData = "";

void initBluetooth() {
    SerialBT.begin("BajaECVT");
}

void readBluetoothData() {
    if (!SerialBT.connected())
        return;

    while (SerialBT.available() > 0) {
        char c = SerialBT.read();

        if (c == ',') {
            processBluetoothData(incomingBluetoothData);
            incomingBluetoothData = "";
        } else if (c != '\n') {
            incomingBluetoothData += c;
        }
    }
}

void processBluetoothData(String data) {
    int dataIndex = data.indexOf(':');

    if (dataIndex != -1) {
        String item = data.substring(0, dataIndex);

        if (item == "RPM") {
            sensors.engineRPM = data.substring(dataIndex + 1).toInt();
        } else if (item == "Throttle") {
            sensors.throttle = (data.substring(dataIndex + 1).toInt()) / 4095.0;
        } else if (item == "Launch") {
            // launchActive = data.substring(dataIndex + 1).toInt();
        } else if (item == "Helix") {
            sensors.helix = data.substring(dataIndex + 1).toInt();
        }
    }
}

void exportBluetoothData() {
    // SerialBT.print("Battery:");  // 0-1
    // SerialBT.print(batPercent);
    // SerialBT.print(",");

    SerialBT.print("RPM:");  // 0-1
    SerialBT.print(sensors.engineRPM);
    SerialBT.print(",");

    // SerialBT.print("Launch:");  // 0-1
    // SerialBT.print(launchActive);
    // SerialBT.print(",");

    SerialBT.print("Brake:");  // 0-1
    SerialBT.print(sensors.brake);
    SerialBT.print(",");

    SerialBT.print("Throttle:");  // 0-1
    SerialBT.print(sensors.throttle);
    SerialBT.print(",");

    SerialBT.print("Helix:");  // 0-1
    SerialBT.print(sensors.helix);
    SerialBT.println(",");
}