#include <WiFi.h>
#include <esp_now.h>
#include <BluetoothSerial.h>

BluetoothSerial BTSerial;
bool isMaster = false;
uint8_t partnerMAC[6] = {0};
esp_now_peer_info_t peerInfo;

typedef struct __attribute__((packed)) {
    uint32_t timestamp;  // For latency calculation
    uint16_t seq_num;    // Sequence number
    bool is_ack;         // ACK flag
    char message[32];    // Payload
} DataPacket;

DataPacket txPacket, rxPacket;
uint16_t packetCounter = 0;
uint32_t lastSendTime = 0;
float latencyEMA = 0;   // Exponential Moving Average of latency
const float alpha = 0.1; // Smoothing factor

// Corrected callback signature
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    memcpy(&rxPacket, data, sizeof(rxPacket));
    
    if (!rxPacket.is_ack) {
        // Handle received message
        Serial.printf("[RX] Seq: %d | Message: %s\n", rxPacket.seq_num, rxPacket.message);
        BTSerial.println(rxPacket.message);

        // Send ACK
        DataPacket ack = {
            .timestamp = rxPacket.timestamp,
            .seq_num = rxPacket.seq_num,
            .is_ack = true,
            .message = "ACK"
        };
        esp_now_send(info->src_addr, (uint8_t *)&ack, sizeof(ack));
    } else {
        // Calculate latency using sender's timestamp
        uint32_t rtt = millis() - rxPacket.timestamp;
        latencyEMA = (alpha * rtt) + ((1 - alpha) * latencyEMA);
        
        Serial.printf("[Latency] RTT: %ums | EMA: %.1fms\n", 
                     rtt, latencyEMA);
    }
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[Error] Delivery failed");
    }
}

void setupESPNow() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) ESP.restart();
    
    esp_now_register_recv_cb(OnDataRecv);  // Now matches the correct signature
    esp_now_register_send_cb(OnDataSent);
}

void setupRoles() {
    // Role detection logic from previous implementations
    // ... (use your existing role setup code here)
    
    // After role detection:
    memcpy(peerInfo.peer_addr, partnerMAC, 6);
    peerInfo.channel = 1;
    esp_now_add_peer(&peerInfo);
}

void sendMessage() {
    txPacket.timestamp = millis();
    txPacket.seq_num = packetCounter++;
    txPacket.is_ack = false;
    snprintf(txPacket.message, sizeof(txPacket.message), 
            "Packet %d", packetCounter);
    
    esp_now_send(partnerMAC, (uint8_t *)&txPacket, sizeof(txPacket));
    lastSendTime = millis();
}

void setup() {
    Serial.begin(115200);
    BTSerial.begin("ESP32_Latency");
    setupRoles();
    setupESPNow();
}

void loop() {
    static uint32_t lastSend = 0;
    
    // Send test packet every 2 seconds
    if (millis() - lastSend > 2000) {
        sendMessage();
        lastSend = millis();
    }

    // Handle Bluetooth input
    if (BTSerial.available()) {
        String input = BTSerial.readString();
        input.trim();
        if (input.length() > 0) {
            sendMessage();
        }
    }
}