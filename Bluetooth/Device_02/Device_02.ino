#include <WiFi.h>
#include <esp_now.h>
#include <BluetoothSerial.h>

BluetoothSerial BTSerial;
bool isMaster = false;
bool masterFound = false;
uint8_t masterMAC[6];  // Store Master's MAC Address
uint8_t slaveMAC[6];   // Store Slave's MAC Address
esp_now_peer_info_t peerInfo;

typedef struct {
    char message[32];
} DataPacket;

DataPacket dataPacket;

// ESP-NOW callback for received messages
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    Serial.print("Received from ESP32: ");
    Serial.println((char*)data);

    // Store Slave MAC in Master Mode
    if (isMaster) {
        memcpy(slaveMAC, info->src_addr, 6);
        esp_now_add_peer(&peerInfo);
    }

    // Forward data to Bluetooth
    BTSerial.println((char*)data);

    // Forward data to Slave if Master
    if (isMaster) {
        Serial.println("Forwarding data back to Slave...");
        esp_now_send(slaveMAC, data, len);
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
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Master added as ESP-NOW peer.");
    } else {
        Serial.println("Failed to add Master as ESP-NOW peer.");
    }
}

// Register Slave Peer
void registerSlavePeer() {
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, slaveMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Slave added as ESP-NOW peer.");
    } else {
        Serial.println("Failed to add Slave as ESP-NOW peer.");
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
        // No Master found → Become Master
        isMaster = true;
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32_TWS_Master");
        Serial.println("BECOMING MASTER");
        BTSerial.begin("ESP32_Master");

        initESPNow(); // Initialize ESP-NOW for Master
        Serial.println("ESP-NOW Master Ready");
    } else {
        // Master found → Become Slave
        isMaster = false;
        WiFi.mode(WIFI_STA);
        WiFi.begin("ESP32_TWS_Master");
        Serial.println("BECOMING SLAVE");
        BTSerial.begin("ESP32_Slave");

        int retryCount = 0;
        while (WiFi.status() != WL_CONNECTED && retryCount < 10) {
            delay(500);
            Serial.print(".");
            retryCount++;
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Connected to Master.");
        } else {
            Serial.println("Failed to connect, promoting to Master.");
            assignRole();
            return;
        }

        initESPNow(); // Initialize ESP-NOW for Slave
        registerMasterPeer();
        Serial.println("ESP-NOW Slave Ready");
    }
}

// Send data over ESP-NOW
void sendData(const char* msg) {
    if (!isMaster && !esp_now_is_peer_exist(peerInfo.peer_addr)) {
        Serial.println("No valid peer. Cannot send data.");
        return;
    }
    strcpy(dataPacket.message, msg);
    
    if (isMaster) {
        esp_now_send(slaveMAC, (uint8_t*)&dataPacket, sizeof(dataPacket));
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

    // Check Bluetooth input
    if (BTSerial.available()) {
        String receivedBT = BTSerial.readString();
        Serial.print("From Mobile: ");
        Serial.println(receivedBT);

        sendData(receivedBT.c_str()); // Forward via ESP-NOW
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
