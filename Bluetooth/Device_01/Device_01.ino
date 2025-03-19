#include <WiFi.h>
#include <esp_now.h>
#include <BluetoothSerial.h>

BluetoothSerial BTSerial;
bool isMaster = false;
bool masterFound = false;
uint8_t masterMAC[6];  // Store Master's MAC Address
esp_now_peer_info_t peerInfo;

// Master variables to track connected slaves
#define MAX_SLAVES 5
uint8_t slaveMACs[MAX_SLAVES][6];
int numSlaves = 0;

typedef struct {
    char message[32];
} DataPacket;

DataPacket dataPacket;

// ESP-NOW callback for received messages
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    Serial.print("Received from ESP32: ");
    Serial.println((char*)data);
    BTSerial.println((char*)data); // Forward to Bluetooth
}

// Scan for Master ESP32
void scanForMaster() {
    WiFi.scanDelete();
    int8_t n = WiFi.scanNetworks();
    masterFound = false;
    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid == "ESP32_TWS_Master") {
            masterFound = true;
            WiFi.BSSID(i, masterMAC);
            break;
        }
    }
}

// Register ESP-NOW Peer for Slave
void registerMasterPeer() {
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, masterMAC, 6);
    peerInfo.channel = WiFi.channel(); // Use the connected channel
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Master added as ESP-NOW peer.");
    } else {
        Serial.println("Failed to add Master as ESP-NOW peer.");
    }
}

// Assign Master/Slave role
void assignRole() {
    // Cleanup previous configurations
    esp_now_deinit();
    WiFi.disconnect(true); // Disconnect and disable STA
    delay(100);
    WiFi.mode(WIFI_OFF);
    delay(100);

    scanForMaster();

    if (!masterFound) {
        // Become Master
        isMaster = true;
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32_TWS_Master", nullptr, 1); // Set channel 1
        Serial.println("BECOMING MASTER");
        BTSerial.begin("ESP32_Master");

        // Register WiFi event handlers for AP
        WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
            if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
                uint8_t* mac = info.sta_connected.mac;
                Serial.print("Slave connected: ");
                char macStr[18];
                snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                Serial.println(macStr);

                // Add slave as ESP-NOW peer
                esp_now_peer_info_t peer;
                memset(&peer, 0, sizeof(peer));
                memcpy(peer.peer_addr, mac, 6);
                peer.channel = 1;
                peer.encrypt = false;
                if (esp_now_add_peer(&peer) == ESP_OK) {
                    if (numSlaves < MAX_SLAVES) {
                        memcpy(slaveMACs[numSlaves], mac, 6);
                        numSlaves++;
                        Serial.println("Slave added as peer.");
                    }
                } else {
                    Serial.println("Failed to add slave peer.");
                }
            } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
                uint8_t* mac = info.sta_disconnected.mac;
                Serial.print("Slave disconnected: ");
                char macStr[18];
                snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                Serial.println(macStr);

                // Remove from slave list
                for (int i = 0; i < numSlaves; i++) {
                    if (memcmp(slaveMACs[i], mac, 6) == 0) {
                        for (int j = i; j < numSlaves - 1; j++) {
                            memcpy(slaveMACs[j], slaveMACs[j+1], 6);
                        }
                        numSlaves--;
                        break;
                    }
                }
                esp_now_del_peer(mac);
            }
        });

        // Initialize ESP-NOW
        if (esp_now_init() != ESP_OK) {
            Serial.println("ESP-NOW init failed");
            return;
        }

        esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status) {
            Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message Sent" : "Send Failed");
        });
        esp_now_register_recv_cb(OnDataRecv); // Master receives data

        Serial.println("ESP-NOW Master Ready");
    } else {
        // Become Slave
        isMaster = false;
        WiFi.mode(WIFI_STA);
        Serial.println("BECOMING SLAVE");
        BTSerial.begin("ESP32_Slave");
        WiFi.begin("ESP32_TWS_Master");

        int retryCount = 0;
        while (WiFi.status() != WL_CONNECTED && retryCount < 20) { // Increased retries
            delay(500);
            Serial.print(".");
            retryCount++;
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Connected to Master.");
            // Register WiFi event to handle disconnection
            WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
                if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
                    Serial.println("Disconnected from Master. Reassigning role...");
                    assignRole();
                }
            }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
        } else {
            Serial.println("Failed to connect, promoting to Master.");
            assignRole();
            return;
        }

        // Initialize ESP-NOW
        if (esp_now_init() != ESP_OK) {
            Serial.println("ESP-NOW init failed");
            return;
        }

        esp_now_register_recv_cb(OnDataRecv);
        registerMasterPeer();

        Serial.println("ESP-NOW Slave Ready");
    }
}

// Send data over ESP-NOW
void sendData(const char* msg) {
    strcpy(dataPacket.message, msg);
    if (isMaster) {
        // Send to all connected slaves
        for (int i = 0; i < numSlaves; i++) {
            esp_now_send(slaveMACs[i], (uint8_t*)&dataPacket, sizeof(dataPacket));
        }
    } else {
        esp_now_send(peerInfo.peer_addr, (uint8_t*)&dataPacket, sizeof(dataPacket));
    }
    Serial.print("Sent: ");
    Serial.println(msg);
}

void setup() {
    Serial.begin(115200);
    assignRole();
}

void loop() {
    // Check Bluetooth input
    if (BTSerial.available()) {
        String receivedBT = BTSerial.readString();
        Serial.print("From Mobile: ");
        Serial.println(receivedBT);
        sendData(receivedBT.c_str());
    }

    // Check Serial Monitor input
    if (Serial.available()) {
        String userInput = Serial.readString();
        Serial.print("From Serial Monitor: ");
        Serial.println(userInput);
        BTSerial.println(userInput);
        sendData(userInput.c_str());
    }
}