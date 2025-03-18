#include <WiFi.h>
#include <esp_now.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
bool isBluetoothConnected = false;
bool espNowPaired = false;
uint8_t peerAddress[6] = {0};  
unsigned long lastScanTime = 0;
const int scanInterval = 5000; 

typedef struct Message {
    char data[100];
} Message;

Message message;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&message, incomingData, len);
    Serial.print("Received via ESP-NOW: ");
    Serial.println(message.data);

    if (isBluetoothConnected) {
        SerialBT.println(message.data);
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("ESP-NOW Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");

    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("Resending message...");
        esp_now_send(peerAddress, (uint8_t *)&message, sizeof(message));
    }
}


void printMacAddress() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("ESP32 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Initialize ESP-NOW
void initESPNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed!");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);
}


void scanForESP32() {
    if (millis() - lastScanTime < scanInterval) return;
    lastScanTime = millis();
    
    if (espNowPaired) return;  

    Serial.println("Scanning for ESP32 devices...");
    int numNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numNetworks; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.startsWith("ESP32_")) { 
            Serial.print("Found ESP32: ");
            Serial.println(ssid);
            
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

// Check Bluetooth Connection
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

void setup() {
    Serial.begin(115200);
    printMacAddress();

    
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("ESP32_DISCOVERABLE");
    WiFi.setSleep(false);  

    // Start Bluetooth
    SerialBT.begin("ESP32_Bluetooth");
    Serial.println("Bluetooth started. Waiting for mobile connection...");

    
    initESPNow();

    
    scanForESP32();
}

void loop() {
    checkBluetoothConnection();
    scanForESP32(); 

    
    if (isBluetoothConnected) {
        while (SerialBT.available()) {
            String receivedData = SerialBT.readString();
            Serial.print("Received from Bluetooth: ");
            Serial.println(receivedData);

            strncpy(message.data, receivedData.c_str(), sizeof(message.data));

            
            if (espNowPaired) {
                esp_now_send(peerAddress, (uint8_t *)&message, sizeof(message));
            }

           
            Serial.println("Forwarding to Serial: " + receivedData);
        }
    }

    
    if (Serial.available()) {
        String serialData = Serial.readString();
        Serial.print("Received from Serial: ");
        Serial.println(serialData);

        
        if (isBluetoothConnected) {
            SerialBT.println(serialData);
        }

        
        if (espNowPaired) {
            strncpy(message.data, serialData.c_str(), sizeof(message.data));
            esp_now_send(peerAddress, (uint8_t *)&message, sizeof(message));
        }
    }

    delay(100);
}
