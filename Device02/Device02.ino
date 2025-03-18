#include <WiFi.h>
#include <esp_now.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
bool isBluetoothConnected = false;
bool espNowPaired = false;
uint8_t peerAddress[6] = {0};  
unsigned long lastSyncTime = 0;
const int syncInterval = 5000;  // Sync every 5 seconds
const int scanInterval = 10000;  

typedef struct Message {
    char data[100];
    unsigned long timestamp;  
} Message;

Message message;

typedef struct SyncMessage {
    unsigned long masterTime;
} SyncMessage;

SyncMessage syncMsg;

bool isMaster = false;  
unsigned long timeOffset = 0;  

void syncClocks(unsigned long masterTimestamp) {
    unsigned long receivedTime = millis();
    timeOffset = masterTimestamp + ((receivedTime - masterTimestamp) / 2) - receivedTime;
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (len == sizeof(SyncMessage)) {
        memcpy(&syncMsg, incomingData, len);
        syncClocks(syncMsg.masterTime);
        return;
    }
    
    memcpy(&message, incomingData, len);
    unsigned long receivedTime = millis() + timeOffset;  

    unsigned long latency = receivedTime - (message.timestamp + timeOffset);
    Serial.print("Received via ESP-NOW: ");
    Serial.println(message.data);
    Serial.print("Latency: ");
    Serial.print(latency);
    Serial.println(" ms");

    if (isBluetoothConnected) {
        SerialBT.println(message.data);
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("ESP-NOW Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");

    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("Resending message...");
        delay(10);
        esp_now_send(peerAddress, (uint8_t *)&message, sizeof(message));
    }
}

void printMacAddress() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("ESP32 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void initESPNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed!");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);
}

void scanForESP32() {
    if (espNowPaired || millis() - lastSyncTime < scanInterval) return;
    lastSyncTime = millis();

    Serial.println("Scanning for ESP32 devices...");
    int numNetworks = WiFi.scanNetworks(false, false);

    for (int i = 0; i < numNetworks; i++) {
        if (WiFi.SSID(i).startsWith("ESP32_")) {
            Serial.print("Found ESP32: ");
            Serial.println(WiFi.SSID(i));

            WiFi.BSSID(i, peerAddress);
            Serial.printf("ESP32 MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          peerAddress[0], peerAddress[1], peerAddress[2], 
                          peerAddress[3], peerAddress[4], peerAddress[5]);

            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, peerAddress, 6);
            peerInfo.channel = 1;
            peerInfo.encrypt = false;

            if (esp_now_add_peer(&peerInfo) == ESP_OK) {
                espNowPaired = true;
                Serial.println("ESP-NOW Paired Successfully!");
            }
            break;
        }
    }
    WiFi.scanDelete();
}

void sendSyncMessage() {
    syncMsg.masterTime = millis();
    esp_now_send(peerAddress, (uint8_t *)&syncMsg, sizeof(syncMsg));
}

void checkBluetoothConnection() {
    if (SerialBT.hasClient()) {
        if (!isBluetoothConnected) {
            Serial.println("Bluetooth device connected!");
            isBluetoothConnected = true;
        }
    } else {
        if (isBluetoothConnected) {
            Serial.println("Bluetooth device disconnected!");
            isBluetoothConnected = false;
        }
    }
}

void BluetoothTask(void *pvParameters) {
    while (1) {
        checkBluetoothConnection();
        if (isBluetoothConnected && SerialBT.available()) {
            String receivedData = SerialBT.readStringUntil('\n');  
            Serial.print("Received from Bluetooth: ");
            Serial.println(receivedData);

            strncpy(message.data, receivedData.c_str(), sizeof(message.data));
            message.timestamp = millis() + timeOffset;  

            if (espNowPaired) {
                esp_now_send(peerAddress, (uint8_t *)&message, sizeof(message));
            }
        }
        vTaskDelay(10);  
    }
}

void SerialTask(void *pvParameters) {
    while (1) {
        if (Serial.available()) {
            String serialData = Serial.readStringUntil('\n');  
            Serial.print("Received from Serial: ");
            Serial.println(serialData);

            if (isBluetoothConnected) {
                SerialBT.println(serialData);
            }

            if (espNowPaired) {
                strncpy(message.data, serialData.c_str(), sizeof(message.data));
                message.timestamp = millis() + timeOffset;  
                esp_now_send(peerAddress, (uint8_t *)&message, sizeof(message));
            }
        }
        vTaskDelay(10);  
    }
}

void WiFiScanTask(void *pvParameters) {
    while (1) {
        scanForESP32();
        vTaskDelay(5000);  
    }
}

void TimeSyncTask(void *pvParameters) {
    while (1) {
        if (isMaster && espNowPaired) {
            sendSyncMessage();
        }
        vTaskDelay(syncInterval / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    printMacAddress();

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("ESP32_DISCOVERABLE");
    WiFi.setSleep(false);  

    SerialBT.begin("ESP32_Bluetooth");
    Serial.println("Bluetooth started. Waiting for connection...");

    initESPNow();

    xTaskCreatePinnedToCore(BluetoothTask, "Bluetooth Task", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(SerialTask, "Serial Task", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(WiFiScanTask, "WiFi Scan Task", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TimeSyncTask, "Time Sync Task", 2048, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelay(1);
}
