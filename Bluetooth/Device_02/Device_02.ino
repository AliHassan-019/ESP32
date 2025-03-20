#include <WiFi.h>
#include <esp_now.h>
#include <BluetoothSerial.h>

BluetoothSerial BTSerial;
bool isMaster = false;
bool masterFound = false;
uint8_t masterMAC[6];  // Store Master's MAC Address
uint8_t slaveMAC[6] = {0};  // Store Slave's MAC Address (for Master)
esp_now_peer_info_t peerInfo;

typedef struct {
    char message[32];
} DataPacket;

DataPacket dataPacket;

// ESP-NOW callback for received messages
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    Serial.print("Received from ESP32: ");
    Serial.println((char*)data);
    BTSerial.println((char*)data); // Forward to Bluetooth

    if (isMaster) {
        // Store slave's MAC if unknown
        if (memcmp(slaveMAC, info->src_addr, 6) != 0) {
            memcpy(slaveMAC, info->src_addr, 6); 

            if (!esp_now_is_peer_exist(slaveMAC)) {
                esp_now_peer_info_t peer;
                memset(&peer, 0, sizeof(peer));
                memcpy(peer.peer_addr, slaveMAC, 6);
                peer.channel = 1;
                peer.encrypt = false;
                
                if (esp_now_add_peer(&peer) == ESP_OK) {
                    Serial.println("Added Slave as peer.");
                } else {
                    Serial.println("Failed to add Slave.");
                }
            }
        }
    }
}

// ESP-NOW callback for sent messages
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("ESP-NOW Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Scan for Master ESP32
void scanForMaster() {
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

// Register ESP-NOW Peer
void registerMasterPeer() {
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, masterMAC, 6);
    peerInfo.channel = 1; 
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Master added as ESP-NOW peer.");
    } else {
        Serial.println("Failed to add Master as ESP-NOW peer.");
    }
}

// Initialize ESP-NOW
void initESPNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Initialization Failed!");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
}

// Assign Master/Slave role
void assignRole() {
    scanForMaster();

    if (!masterFound) {
        // Become Master
        isMaster = true;
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32_TWS_Master", nullptr, 1);
        Serial.println("BECOMING MASTER");
        BTSerial.begin("ESP32_Master");

        initESPNow();
        Serial.println("ESP-NOW Master Ready");
    } else {
        // Become Slave
        isMaster = false;
        WiFi.mode(WIFI_STA);
        WiFi.begin("ESP32_TWS_Master", nullptr, 1);
        Serial.println("BECOMING SLAVE");
        BTSerial.begin("ESP32_Slave");

        int retryCount = 0;
        while (WiFi.status() != WL_CONNECTED && retryCount < 10) {
            delay(500);
            Serial.print(".");
            retryCount++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected to Master");
            initESPNow();
            registerMasterPeer();
            Serial.println("ESP-NOW Slave Ready");

            // Send initial message to master
            sendData("Hello from Slave");
        } else {
            Serial.println("Failed to connect, promoting to Master");
            assignRole();
        }
    }
}

// Send data over ESP-NOW
void sendData(const char* msg) {
    strcpy(dataPacket.message, msg);
    bool validMAC = false;

    if (isMaster) {
        // Ensure we have a valid slave MAC address
        for (int i = 0; i < 6; i++) {
            if (slaveMAC[i] != 0) {
                validMAC = true;
                break;
            }
        }
        
        if (validMAC) {
            esp_now_send(slaveMAC, (uint8_t*)&dataPacket, sizeof(dataPacket));
        } else {
            Serial.println("No valid MAC address for sending!");
            return;
        }
    } else {
        esp_now_send(peerInfo.peer_addr, (uint8_t*)&dataPacket, sizeof(dataPacket));
    }

    Serial.print("Sent: ");
    Serial.println(msg);
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    assignRole();
}

void loop() {
    // Reassign role if master disconnects
    if (!isMaster && WiFi.status() != WL_CONNECTED) {
        Serial.println("Master lost, switching to Master...");
        assignRole();
    }

    // Handle Bluetooth input
    if (BTSerial.available()) {
        String receivedBT = BTSerial.readString();
        Serial.print("From Mobile: ");
        Serial.println(receivedBT);
        sendData(receivedBT.c_str());
    }

    // Handle Serial input
    if (Serial.available()) {
        String userInput = Serial.readString();
        Serial.print("From Serial Monitor: ");
        Serial.println(userInput);
        BTSerial.println(userInput); // Send to Mobile
        sendData(userInput.c_str());
    }
}
