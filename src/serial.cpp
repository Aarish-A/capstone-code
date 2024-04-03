#include "serial.h"

// Global Extern Variables
HardwareSerial Onboard(0);
String incomingOnboardData = "";

void initSerial() {
    Onboard.begin(115200, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
    Wire.begin();
}

// Function to read onboard data from UART
void readOnboardData() {
    while (Onboard.available() > 0) {
        char c = Onboard.read();

        if (c == ',') {
            processOnboardData(incomingOnboardData);
            incomingOnboardData = "";
        } else if (c != '\n') {
            incomingOnboardData += c;
        }
    }
}

// Function to process onboard data received from UART
void processOnboardData(String data) {
    int dataIndex = data.indexOf(':');

    if (dataIndex != -1) {
        String item = data.substring(0, dataIndex);

        if (item == "RPM") {
            sensors.engineRPM = data.substring(dataIndex + 1).toInt();
        } else if (item == "Throttle") {
            sensors.throttle = potRead(data.substring(dataIndex + 1).toFloat());
        } else if (item == "Launch") {
            // launchActive = data.substring(dataIndex + 1).toInt();
        }
    }
}
