/*********************************************************************
LoRa Receiver -> BLE Transmitter
*/

#include<Arduino.h>
#include <bluefruit.h>
//#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <LoRa.h>

// //define the pins used by the transceiver module
// #define ss A5   //a5
// #define rst A4  //a4
// #define dio0 5  //d5

// Pin Configuration for LoRa
#define ss 5
#define rst 14
#define dio0 2

#define DEBUG_SERIAL 0
//Serial.print and DEBUG_PRINTLN replaced with a macro
#define DEBUG_PRINT(x) do { if (DEBUG_SERIAL) Serial.print(x); } while (0)
#define DEBUG_PRINTLN(x) do { if (DEBUG_SERIAL) Serial.println(x); } while (0)
#define DEBUG_PRINTF(...) do { if (DEBUG_SERIAL) Serial.printf(__VA_ARGS__); } while (0)


//Adafruit_DotStar strip(1, 8, 6, DOTSTAR_BRG);

#define ADV_TIMEOUT   3 // seconds. Set this higher to automatically stop advertising after a time


// The following code is for setting a name based on the actual device MAC address
// Where to go looking in memory for the MAC
typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)
#define MAC_ADDRESS_HIGH  (*(pREG32 (0x100000a8)))
#define MAC_ADDRESS_LOW   (*(pREG32 (0x100000a4)))
int counter = 0;
bool flag = true;


void startAdv();

char nibble_to_hex(uint8_t nibble) {  // convert a 4-bit nibble to a hexadecimal character
 nibble &= 0xF;
 return nibble > 9 ? nibble - 10 + 'A' : nibble + '0';
}


void byte_to_str(char* buff, uint8_t val) {  // convert an 8-bit byte to a string of 2 hexadecimal characters
 buff[0] = nibble_to_hex(val >> 4);
 buff[1] = nibble_to_hex(val);
}

/**
* Callback invoked when advertising is stopped by timeout
*/
void adv_stop_callback(void)
{
 DEBUG_PRINTLN("Advertising time passed, advertising will now stop.");
 counter += 1;
 if (counter >= 10) counter = 0;
 flag = true;
}


void setup()
{
  // BLE setup:
  Serial.begin(9600);
  while ( !Serial ) delay(10);
  //  strip.begin();
  //  strip.show();
  DEBUG_PRINTLN("TreeTap Node 2");
  DEBUG_PRINTLN("LoRa Receiver -> BLE Transmitter");
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
  // Serial.println();

  //Serial.println("----------------------------------------\n");

  Bluefruit.begin();
  Bluefruit.setTxPower(0);    // Check bluefruit.h for supported values

  char ble_name[14] = "TreeTap"; // Null-terminated string must be 1 longer than you set it, for the null
  uint32_t addr_low  = MAC_ADDRESS_LOW;


  //  // Fill in the XXXX in ble_name
  //  byte_to_str(&ble_name[9], (addr_low >> 8) & 0xFF);
  //  byte_to_str(&ble_name[11], addr_low & 0xFF);
  //  // Set the name we just made
  Bluefruit.setName(ble_name);


  // LoRa setup
  //433E6 for Asia
  //868E6 for Europe
  //915E6 for North America
  //setup LoRa transceiver module
  //LoRa.setPins(ss, rst, dio0);
  LoRa.setPins(ss, rst);
  while (!LoRa.begin(915E6)) {
    DEBUG_PRINTLN("Waitin for LoRa initialization");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  // Increase spreading factor for better range/reliability
  LoRa.setSpreadingFactor(12);  // Maximum spreading factor
  // Enable coding rate for error correction
  LoRa.setCodingRate4(8);  // Higher coding rate means more redundancy
  DEBUG_PRINTLN("LoRa Initializing OK!");
}


void startAdv(uint8_t* data) // The advertised data could just be a UID
{
  //if (data.indexOf("hi") != -1) {
    //DEBUG_PRINTLN("\n************Alert received!!************");
  //}
  //int   ArrayLength = length+1;    //The +1 is for the 0x00h Terminator
  //char  CharArray[ArrayLength];
  //data.toCharArray(CharArray,ArrayLength);

  DEBUG_PRINT("Hexadecimal representation: ");
  int length = 11;
  for (unsigned int i = 0; i < length; i++) {
    char c = data[i]; // Get each character
    //DEBUG_PRINT(c, HEX);   // Print character in hexadecimal
    //DEBUG_PRINT(" ");      // Add space between hex values
  }

  // Advertising packet
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.Advertising.addName();


  uint8_t manufacturer_data[11]; // 2 bytes for company ID + 11 bytes payload
  memcpy(&manufacturer_data, data, 11); // Copy payload, manufacturer data included
  // Convert and print each character in hexadecimal
  DEBUG_PRINTLN();
  Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, manufacturer_data, sizeof(manufacturer_data));


  //Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, (uint8_t*)id, sizeof(id));
  // Tell the BLE device we want to send our name in a ScanResponse if asked.
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
  * - Enable auto advertising if disconnected
  * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
  * - Timeout for fast mode is 30 seconds
  * - Start(timeout) with timeout = 0 will advertise forever (until connected)
  *
  * For recommended advertising interval
  * https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(2);      // number of seconds in fast mode
  Bluefruit.Advertising.start(ADV_TIMEOUT);      // Stop advertising entirely after ADV_TIMEOUT seconds
  //Bluefruit.Advertising.start(0);    

}


void loop() {
  // try to parse LoRa packet
  if(flag){ //only read LORA if not sending Bluetooth
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      // DEBUG_PRINTLN(packetSize);
      // <Possible reformating on end point>

      DEBUG_PRINTLN("Received packet");
      // read packet and transmit it using BLE
      uint8_t LoRaData[11];
      for(int i=0; i<11; i++){
        LoRaData[i] = LoRa.read();
      }
      startAdv(LoRaData);
      //SOS Serialcom:
      for(int i=0;i<11;i++){
        Serial.printf("%02X ", LoRaData[i]);
      }
      Serial.print("\n");
      flag = false;
    }  
    // print RSSI of packet
    // Serial.print("' with RSSI ");
    // DEBUG_PRINTLN(LoRa.packetRssi());
  }
}
