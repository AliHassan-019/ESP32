#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
bool isMaster = false;
bool isConnected = false;
const char* other_esp32_mac = "A0:B7:65:04:A3:A2";

void setup() {
    Serial.begin(115200);
    delay(1000);
    SerialBT.begin("NIURA!", false);
    Serial.println("Device Ready - Waiting for phone connection...");
}

void loop() {
    if (SerialBT.hasClient()) {
        if (!isConnected) {
            Serial.println("Phone Connected!");
            isConnected = true;
        }

        if (!isMaster) {
            Serial.println("Searching for other pair!");
            if (SerialBT.connect(other_esp32_mac)) {
                Serial.println("Paired to other device, switching to Masters mode");
                isMaster = true;
            } else {
                Serial.println("No other pair detected!");
            }
        }
    } else { 
        if (isConnected) {
            Serial.println("Phone disconnected! Restarting Bluetooth...");
            isConnected = false;
            isMaster = false;
            SerialBT.end();
            delay(2000);
            SerialBT.begin("Niura", false);
            Serial.println("Device Ready - Waiting for phone connection...");
        }
    }

    while (SerialBT.available()) {
        char receivedChar = SerialBT.read(); 
        Serial.print(receivedChar); 
        SerialBT.write(receivedChar); 
    }

    while (Serial.available()) {
        char sendChar = Serial.read(); 
        SerialBT.write(sendChar); 
        SerialBT.flush(); 
    }

    delay(50); 
}
