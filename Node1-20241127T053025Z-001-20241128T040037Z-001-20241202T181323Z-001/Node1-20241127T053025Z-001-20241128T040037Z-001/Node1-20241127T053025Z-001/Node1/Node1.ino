#include <Arduino.h>
#include <bluefruit.h>
#include <SPI.h>
#include <LoRa.h>


/****************************
BLE Receiver -> LoRa Transmitter
nrf52840 Feather Express: https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather/pinouts
*/

#define TREETAP_ID 

//define the pins used by the transceiver module FEATHER
#define ss A5   //a5
#define rst A4  //a4

// // Pin Configuration for LoRa
// #define ss 5
// #define rst 14
// #define dio0 2

#define DEBUG_SERIAL 1
//Serial.print and DEBUG_PRINTLN replaced with a macro
#define DEBUG_PRINT(x) do { if (DEBUG_SERIAL) Serial.print(x); } while (0)
#define DEBUG_PRINTLN(x) do { if (DEBUG_SERIAL) Serial.println(x); } while (0)
#define DEBUG_PRINTF(...) do { if (DEBUG_SERIAL) Serial.printf(__VA_ARGS__); } while (0)
#define DEBUG_PRINTBUFFER(x) do { if (DEBUG_SERIAL) Serial.printBuffer(x); } while (0)

// https://adafruit.github.io/arduino-board-index/package_adafruit_index.json

typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)
#define MAC_ADDRESS_HIGH  (*(pREG32 (0x100000a8)))
#define MAC_ADDRESS_LOW   (*(pREG32 (0x100000a4)))


/* For a list of EIR data types see:
*    https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
*    Matching enum: cores/nRF5/SDK/components/softdevice/s132/headers/ble_gap.h */

void printUuid128List(uint8_t* buffer, uint8_t len);
void printUuid16List(uint8_t* buffer, uint8_t len);
void scan_callback(ble_gap_evt_adv_report_t* report);
void extractFirstFour(uint8_t* buffer, char* outString);
void lora_relay(uint8_t* buffer, uint8_t len);
void scan_callback(ble_gap_evt_adv_report_t* report);
void reverseString(char* str);

const char* manufacturer = "CCFA";
const char* name = "TreeTap";
//const char* name = "A47595A8-8121-43D7-8761-A13E67E9BBC0";

void reverseString(char* str) {
  int length = strlen(str);
  int start = 0;
  int end = length - 1;
  
  while (start < end) {
    // Swap characters
    char temp = str[start];
    str[start] = str[end];
    str[end] = temp;
    
    start++;
    end--;
  }
}

void printUuid16List(uint8_t* buffer, uint8_t len)
{
  DEBUG_PRINTF("%14s %s", "16-Bit UUID");
  for(int i=0; i<len; i+=2)
  {
    uint16_t uuid16;
    memcpy(&uuid16, buffer+i, 2);
    DEBUG_PRINTF("%04X ", uuid16);
  }
  DEBUG_PRINTLN();
}

void printUuid128List(uint8_t* buffer, uint8_t len)
{
  (void) len;
  DEBUG_PRINTF("%14s %s", "128-Bit UUID");

  // Print reversed order
  for(int i=0; i<16; i++)
  {
    const char* fm = (i==4 || i==6 || i==8 || i==10) ? "-%02X" : "%02X";
    DEBUG_PRINTF(fm, buffer[15-i]);
  }


  DEBUG_PRINTLN();
}

void extractFirstFour(uint8_t* buffer, char* outString) {
    // Allocate enough space for 2 bytes (4 hex chars + null terminator)
    const size_t numBytes = 2;
    
    // Convert each byte to two hex characters
    for (size_t i = 0; i < numBytes; i++) {
        outString[i*2] = "0123456789ABCDEF"[(buffer[i] >> 4) & 0x0F];  // High nibble
        outString[i*2 + 1] = "0123456789ABCDEF"[buffer[i] & 0x0F];     // Low nibble
    }
    outString[8] = '\0';  // Null terminate
}

void setup()
{
  Serial.begin(9600);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  DEBUG_PRINTLN("TreeTap Node 1");
  DEBUG_PRINTLN("BLE Receiver -> LoRa Transmitter");
  // Read the MAC address from the defined registers
  uint32_t macHigh = MAC_ADDRESS_HIGH;
  uint32_t macLow = MAC_ADDRESS_LOW;

  // Print the MAC address
  // Serial.print("MAC Address: ");
  // Serial.print((macHigh >> 8) & 0xFF, HEX);
  // Serial.print(":");
  // Serial.print(macHigh & 0xFF, HEX);
  // Serial.print(":");
  // Serial.print((macLow >> 24) & 0xFF, HEX);
  // Serial.print(":");
  // Serial.print((macLow >> 16) & 0xFF, HEX);
  // Serial.print(":");
  // Serial.print((macLow >> 8) & 0xFF, HEX);
  // Serial.print(":");
  // Serial.print(macLow & 0xFF, HEX);
  // DEBUG_PRINTLN();

  DEBUG_PRINTLN("------------------------------------\n");

  //setup LoRa transceiver module
  //LoRa.setPins(ss, rst, dio0);
  LoRa.setPins(ss, rst);
  //replace the LoRa.begin(---E-) argument with your location's frequency
  //433E6 for Asia
  //868E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(915E6)) {
    DEBUG_PRINTLN("Waiting for LoRa");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  // Increase spreading factor for better range/reliability
  LoRa.setSpreadingFactor(12);  // Maximum spreading factor
  // Enable coding rate for error correction
  LoRa.setCodingRate4(8); // Higher coding rate means more redundancy
  DEBUG_PRINTLN("LoRa Initializing OK!");

  // Tx power: 2-20dB, standard=17
  // LoRa.setTxPower(17);

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(-40);    // Check bluefruit.h for supported values

  /* Set the device name */
  Bluefruit.setName("TreeTap_N1");  // Replace 1 with your TREETAP_ID

  /* Set the LED interval for blinky pattern on BLUE LED */
  //Bluefruit.autoConnLed(false);

  /* Start Central Scanning
  * - Enable auto scan if disconnected
  * - Filter out packet with a min rssi
  * - Interval = 100 ms, window = 50 ms
  * - Use active scan (used to retrieve the optional scan response adv packet)
  * - Start(0) = will scan forever since no timeout is given
  */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-90);
  //Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE); // only invoke callback if detect bleuart service
  //Bluefruit.Scanner.setInterval(250, 200);       // in units of 0.625 ms
  Bluefruit.Scanner.setInterval(100, 80);  // Reduces to ~62.5 ms interval, 50 ms window
  Bluefruit.Scanner.useActiveScan(true);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds

  DEBUG_PRINTLN("Scanning ...");
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  //DEBUG_PRINTLN("Received BLE ad");
  uint8_t len = 0;
  uint8_t buffer[32];

  if(report->type.connectable){
    Bluefruit.Scanner.resume();
    return;
  }

  /* Parse by Manufacturer Specific Data. Data is printed at the end*/
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
  if (len)
  {
    // MSD debugging:
    char msd[5];
    extractFirstFour(buffer, msd);
    if(!strstr(msd, manufacturer)){
      Bluefruit.Scanner.resume();
      //DEBUG_PRINTLN("NOT MSD");
      return;
    }
    
  }else {
    Bluefruit.Scanner.resume();
    return;
  }

  /* Check Local Name. If parsing by name, uncomment */
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    DEBUG_PRINTF("\n\n\n%14s%s", "COMPLETE NAME: ", buffer);
    if(strstr((char*)buffer, "TreeTap")){
        DEBUG_PRINT("\nERROR: RECEIVED FROM TREETAP NODE");
        Bluefruit.Scanner.resume();
        return;
    }
  }

  /* Check for Manufacturer Specific Data */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
  if (len)
  {
    // MSD debugging:
    // char msd[5];
    // //bufferToHexString(buffer, 4, msd);
    // extractFirstFour(buffer, msd);
    // DEBUG_PRINTF("%s", msd);

    DEBUG_PRINTF("%14s ", "MAN SPEC DATA");
    //Serial.printBuffer(buffer, len, '-');
    DEBUG_PRINTLN();
    DEBUG_PRINTF("%s\n", "RAW DATA");
    for(int i=0; i<len; i++){
      DEBUG_PRINTLN(buffer[i]);
    }
    for(int i=0; i<len; i++){
      DEBUG_PRINTLN(buffer[i]);
    }
    DEBUG_PRINTLN();

    // LoRa transmission
    lora_relay(buffer, len);
    memset(buffer, 0, len);
  }


  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();

}

void lora_relay(uint8_t* buffer, uint8_t len){
  DEBUG_PRINTLN("Sending packet: ");
  LoRa.beginPacket();

  //Print the actual data bytes in hex
  DEBUG_PRINT("Data (hex): ");
  for (size_t i = 0; i < len; i++) {
    DEBUG_PRINTF("%02X ", buffer[i]);
  }
  DEBUG_PRINTLN();
  LoRa.write(buffer, len);
  LoRa.endPacket();
  delay(1);
}


void loop()
{
// nothing to do
}

