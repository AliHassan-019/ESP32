#include <WiFi.h>
#include <esp_now.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
bool isBluetoothConnected = false;
bool isEspNowPaired = false;

// 🔴 Specific ESP32 MAC Address (Peer Device)
uint8_t peerAddress[] = {0xA0, 0xB7, 0x65, 0x04, 0xA3, 0xA2};

typedef struct Message {
    char data[100];
} Message;

Message message;

// Callback when ESP-NOW data is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&message, incomingData, len);
    Serial.print("✅ Received via ESP-NOW: ");
    Serial.println(message.data);

    if (isBluetoothConnected) {
        SerialBT.println(message.data);
    }
}

// Callback when ESP-NOW data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("📤 ESP-NOW Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success ✅" : "Failed ❌");

    if (status == ESP_NOW_SEND_FAIL) {
        Serial.println("⚠️ Retrying transmission...");
        esp_now_send(peerAddress, (uint8_t *)&message, sizeof(message));
    }
}

// Initialize ESP-NOW and add the specific peer
void initESPNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW Init Failed!");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // Add peer (specific ESP32)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;  // Default channel
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        isEspNowPaired = true;
        Serial.println("✅ ESP-NOW Paired with Specific ESP32!");
    } else {
        Serial.println("❌ Failed to pair with ESP32.");
    }
}

// Check Bluetooth Connection
void checkBluetoothConnection() {
    if (SerialBT.hasClient()) {
        if (!isBluetoothConnected) {
            Serial.println("✅ Bluetooth Device Connected!");
            isBluetoothConnected = true;
        }
    } else {
        if (isBluetoothConnected) {
            Serial.println("❌ Bluetooth Device Disconnected!");
            isBluetoothConnected = false;
        }
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);  // Set WiFi mode as Station
    WiFi.setSleep(false);

    // Start Bluetooth
    SerialBT.begin("ESP32_Bluetooth");
    Serial.println("🔵 Bluetooth Started. Waiting for connection...");

    // Initialize ESP-NOW
    initESPNow();
}

void loop() {
    checkBluetoothConnection();

    // Read Bluetooth Data and send via ESP-NOW
    if (isBluetoothConnected) {
        while (SerialBT.available()) {
            String receivedData = SerialBT.readString();
            Serial.print("📩 Received from Bluetooth: ");
            Serial.println(receivedData);

            strncpy(message.data, receivedData.c_str(), sizeof(message.data));

            if (isEspNowPaired) {
                Serial.println("📡 Sending to ESP-NOW Peer...");
                esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&message, sizeof(message));
                if (result != ESP_OK) {
                    Serial.println("❌ ESP-NOW Send Failed!");
                }
            }
        }
    }

    // Read Serial Data and send via ESP-NOW
    if (Serial.available()) {
        String serialData = Serial.readString();
        Serial.print("📩 Received from Serial: ");
        Serial.println(serialData);

        if (isBluetoothConnected) {
            SerialBT.println(serialData);
        }

        strncpy(message.data, serialData.c_str(), sizeof(message.data));

        if (isEspNowPaired) {
            Serial.println("📡 Sending Serial Data via ESP-NOW...");
            esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&message, sizeof(message));
            if (result != ESP_OK) {
                Serial.println("❌ ESP-NOW Send Failed!");
            }
        }
    }
}
