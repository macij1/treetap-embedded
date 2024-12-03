#include <Arduino.h>
#include <bluefruit.h>
#include <SPI.h>
#include <LoRa.h>

// // Pin Configuration for LoRa ITSYBITSY
// #define ss 5
// #define rst 14
// #define dio0 2

// Pin Configuration for LoRa FEATHER
#define ss A5
#define rst A4

// MAC memory map
// The following code is for setting a name based on the actual device MAC address
// Where to go looking in memory for the MAC
typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)
#define MAC_ADDRESS_HIGH  (*(pREG32 (0x100000a8)))
#define MAC_ADDRESS_LOW   (*(pREG32 (0x100000a4)))

#define DEBUG_SERIAL 1
//Serial.print and DEBUG_PRINTLN replaced with a macro
#define DEBUG_PRINT(x) do { if (DEBUG_SERIAL) Serial.print(x); } while (0)
#define DEBUG_PRINTLN(x) do { if (DEBUG_SERIAL) Serial.println(x); } while (0)
#define DEBUG_PRINTF(...) do { if (DEBUG_SERIAL) Serial.printf(__VA_ARGS__); } while (0)
#define DEBUG_PRINTBUFFER(x) do { if (DEBUG_SERIAL) Serial.printBuffer(x); } while (0)

// Network Configuration
#define MANUFACTURER_ID 0xFACC     // Unique identifier for your network
#define MAX_PACKET_SIZE 11         // Maximum packet size for compatibility
#define DEFAULT_TTL 3              // Default Time-To-Live for message relay
#define MAX_NEARBY_DEVICES 20      // Maximum number of nearby BLE devices to track
#define TREETAP_ID 1

void addNearbyDevice(uint8_t* addr, int8_t rssi);
void cleanupNearbyDevices();
void printNearbyDevices();
void initLoRa();
void initBLE();
void scanCallback(ble_gap_evt_adv_report_t* report);
void lora_send(uint8_t* buffer, uint8_t len);
void lora_read();
void update_state(int node, int number);

// Will hold current known state
uint8_t state[10];
    
// Nearby device tracking
struct NearbyDevice {
    uint8_t address[6];  // MAC address
    int8_t rssi;         // Signal strength
    unsigned long lastSeen;
};
    
NearbyDevice nearbyDevices[MAX_NEARBY_DEVICES];
int32_t nearbyDeviceCount = 0;

void addNearbyDevice(uint8_t* addr, int8_t rssi) {
    // Check if device already exists
    for (uint8_t i = 0; i < nearbyDeviceCount; i++) {
        if (memcmp(nearbyDevices[i].address, addr, 6) == 0) {
            // Update existing device
            nearbyDevices[i].rssi = rssi;
            nearbyDevices[i].lastSeen = millis();
            return;
        }
    }

    // Add new device if there's space
    if (nearbyDeviceCount < MAX_NEARBY_DEVICES) {
        memcpy(nearbyDevices[nearbyDeviceCount].address, addr, 6);
        nearbyDevices[nearbyDeviceCount].rssi = rssi;
        nearbyDevices[nearbyDeviceCount].lastSeen = millis();
        nearbyDeviceCount++;
    }
}

void cleanupNearbyDevices() {
    unsigned long currentTime = millis();
    for (uint8_t i = 0; i < nearbyDeviceCount; i++) {
        // Remove devices not seen in last 2 minutes
        if (currentTime - nearbyDevices[i].lastSeen > 120000) {
            // Shift remaining devices
            for (uint8_t j = i; j < nearbyDeviceCount - 1; j++) {
                nearbyDevices[j] = nearbyDevices[j + 1];
            }
            nearbyDeviceCount--;
            i--; // Recheck the current index
        }
    }
}

// Method to print nearby devices (for debugging)
void printNearbyDevices() {
    DEBUG_PRINTLN("Nearby BLE Devices:");
    for (uint8_t i = 0; i < nearbyDeviceCount; i++) {
        DEBUG_PRINTF("Device %d: MAC ", i);
        for (int j = 5; j >= 0; j--) {
            DEBUG_PRINTF("%02X", nearbyDevices[i].address[j]);
            if (j > 0) DEBUG_PRINT(":");
        }
        DEBUG_PRINTF(" | RSSI: %d\n", nearbyDevices[i].rssi);
    }
}

void initLoRa() {
    //LoRa.setPins(ss, rst, dio0);
    LoRa.setPins(ss, rst);
    while (!LoRa.begin(915E6)) {
        DEBUG_PRINTLN("Waiting for LoRa");
        delay(500);
    }
    LoRa.setSyncWord(0xF3);
    LoRa.setSpreadingFactor(12); //TODO: can be adjusted to balance power, speed and reliability
    LoRa.setCodingRate4(8); //TODO: can be adjusted to balance power, speed and reliability
    DEBUG_PRINTLN("\nLoRa Initialized");
}

void initBLE() {
    Bluefruit.begin(0, 1);  // 0 Peripheral, 1 Central
    Bluefruit.setTxPower(-40);
    
    char nodeName[14];
    DEBUG_PRINTF("TreeTap_%d", TREETAP_ID);
    Bluefruit.setName(nodeName);

    // Setup BLE Scanner
    Bluefruit.Scanner.setRxCallback(scanCallback);
    Bluefruit.Scanner.restartOnDisconnect(true);
    Bluefruit.Scanner.filterRssi(-90); //TODO: change back to -120 in outdoor conditions
    Bluefruit.Scanner.setInterval(250, 200); //TODO: can be adjusted to balance power, speed and reliability
    Bluefruit.Scanner.useActiveScan(true);
    Bluefruit.Scanner.start(0);
    DEBUG_PRINTLN("\nBLE Initialized");
}

    // void transmitBLE(NetworkPacket& packet) {
    //     uint8_t manufacturerData[13];
    //     manufacturerData[0] = MANUFACTURER_ID & 0xFF;
    //     manufacturerData[1] = (MANUFACTURER_ID >> 8) & 0xFF;
    //     memcpy(&manufacturerData[2], &packet, sizeof(NetworkPacket));

    //     Bluefruit.Advertising.clearData();
    //     Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    //     Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, manufacturerData, sizeof(manufacturerData));
    //     Bluefruit.Advertising.start(3);  // Advertise for 3 seconds
        
    //     DEBUG_PRINTLN("Broadcasting BLE packet");
    // }

// Static? callback method
void scanCallback(ble_gap_evt_adv_report_t* report) {
    uint8_t buffer[32];
    // Name control:
    if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer))){
        //DEBUG_PRINTF("%14s %s\n", "COMPLETE NAME", buffer);
        if(strstr((char*)buffer, "TreeTap")){
        Bluefruit.Scanner.resume();
        return;
        }
        memset(buffer, 0, sizeof(buffer));
    }
    // Manufacturer Data control, filtering based on ID
    uint8_t len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
    // Call addNearbyDevice through the instance
    addNearbyDevice(report->peer_addr.addr, report->rssi);

    if (len && buffer[0] == (MANUFACTURER_ID & 0xFF) && buffer[1] == ((MANUFACTURER_ID >> 8) & 0xFF)) {
        /****************DEBUG MSD****************/
        // DEBUG_PRINTF("%14s ", "MAN SPEC DATA");
        // Serial.printBuffer(buffer, len, '-');
        // DEBUG_PRINTLN();
        // DEBUG_PRINTF("%s\n", "RAW DATA");
        // for(int i=0; i<len; i++){
        //   DEBUG_PRINTLN(buffer[i]);
        // }
        // for(int i=0; i<len; i++){
        //   DEBUG_PRINTLN(buffer[i]);
        // }
        // DEBUG_PRINTLN();
        //****************DEBUG MSD****************/
    }
    
    Bluefruit.Scanner.resume();
}


void setup() {
    Serial.begin(9600);
    while ( !Serial ) delay(10);
     DEBUG_PRINTF("TreeTap Node: %d\n", TREETAP_ID);
    initLoRa();
    initBLE();
    state[0] = 1 ;
    state[1] = 0 ; // int32: number of devices at device 1
    state[2] = 0 ;
    state[3] = 0 ;
    state[4] = 0 ;
    state[5] = 2 ;
    state[6] = 0 ; // int32: number of devices at device 2
    state[7] = 0 ;
    state[8] = 0 ;
    state[9] = 0 ;

}

void lora_send(uint8_t* buffer, uint8_t len){
  DEBUG_PRINTLN("\nSending packet: ");
  LoRa.beginPacket();
  if(len!=11){
    // DELETE SERIAL DEBUG MSG
    DEBUG_PRINTLN("Error on Lora size");
  }
  //Print the actual data bytes in hex
  DEBUG_PRINT("Data (hex): ");
  for (size_t i = 0; i < len; i++) {
    DEBUG_PRINTF("%02X ", buffer[i]);
  }
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("\nLora tx starts ");
  LoRa.write(buffer, len);
  LoRa.endPacket();
  DEBUG_PRINTLN("\nLora tx ended ");
}

void lora_read(){
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // DEBUG_PRINTLN(packetSize);
        DEBUG_PRINTLN("Received packet");
        // read packet and transmit it using BLE
        uint8_t LoRaData[11];
        for(int i=0; i<11; i++){
            LoRaData[i] = LoRa.read();
        }
        DEBUG_PRINT("\nRECEIVED (hex): ");
        for (size_t i = 0; i < 11; i++) {
          DEBUG_PRINTF("%02X ", LoRaData[i]);
        }
        if (LoRaData[0] > 0){ //ttl
            // TODO: UNCOMMENT WEHN MORE TREETAP NODES
            // lora_send(&buffer, 5)
            
            // Only consider LoRa data about OTHER nodes
            if(TREETAP_ID==2){
                // Reconstruct the int32_t value assuming little-endian format // CHECK THIS
                int count = (int32_t)(LoRaData[2]) | 
                    (int32_t)(LoRaData[3]) << 8 | 
                    (int32_t)(LoRaData[4]) << 16 | 
                    (int32_t)(LoRaData[5]) << 24;
                DEBUG_PRINTF("COUNT: %d", count);
                update_state(1, count);
            }else if(TREETAP_ID==1){
                // Reconstruct the int32_t value assuming little-endian format
                int count = (int32_t)(LoRaData[10]) | 
                    (int32_t)(LoRaData[9]) << 8 | 
                    (int32_t)(LoRaData[8]) << 16 | 
                    (int32_t)(LoRaData[7]) << 24;
                update_state(2, count);
                DEBUG_PRINTF("COUNT: %d", count);
            }
        }
    }  
}

void update_state(int node, int number){
    if(node == 1){
        state[4] = (uint8_t)(number & 0xFF);          // Least significant byte
        state[3] = (uint8_t)((number >> 8) & 0xFF);   
        state[2] = (uint8_t)((number >> 16) & 0xFF);  
        state[1] = (uint8_t)((number >> 24) & 0xFF);  // Most significant byte
    }else if(node==2){
        state[9] = (uint8_t)(number & 0xFF);          // Least significant byte
        state[8] = (uint8_t)((number >> 8) & 0xFF);   
        state[7] = (uint8_t)((number >> 16) & 0xFF);  
        state[6] = (uint8_t)((number >> 24) & 0xFF);  // Most significant byte
    }else{
        DEBUG_PRINTLN("ERROR");
    }
}

void loop() {
    // Periodically clean up nearby devices list
    cleanupNearbyDevices();
    update_state(TREETAP_ID, nearbyDeviceCount);
    // DELETE SERIAL DEBUG MSG
    DEBUG_PRINTLN("\nSTATE: ");
    for (int i = 0; i < sizeof(state); i++) {
        //Serial.write(state[i]);  // Send each byte of the buffer to the DesktopApp
        // DELETE SERIAL DEBUG MSG
        Serial.printf("%02X ", state[i]);
    }
    Serial.printf("\n");
    uint8_t buffer[11];
    buffer[0] = 3; //ttl
    memcpy(buffer+1, state, 10);
    lora_send(buffer, 11); // broadcast new state
    //LoRa.receive();
    lora_read();
    delay(5000);
    //printNearbyDevices();
}