/*----------------------- Libaries ------------------ */
#include <SPI.h>
#include <LoRa.h>

/*-------------------------- LoRa --------------------- */
// LoRa pins
const int csPin = 24;
const int resetPin = 26;
const int irqPin = 25;

// Frquency for LoRa in Hz
const long freq = 4339E5; // 433.9 MHz

int message_rssi;
float message_snr;

/*----------------------- Functions ------------------------- */
// Function that runs waits until data package has been received
void onReceive(int packetSize) {
  // If no package received stop function
  if (packetSize == 0) return;

  // String to save incoming package
  String incoming = "";

  // Read package data and append each charachter to string
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  message_rssi = LoRa.packetRssi();
  message_snr = LoRa.packetSnr();
  // At the end of the message append the signal strength and signal to noise ratio
  incoming += ",";
  incoming += message_rssi;
  incoming += ",";
  incoming += message_snr;
  
  // Print the message to serial monitor
  Serial.println(incoming);
}

/*------------- Setup --------------*/
void setup() {
  // Begin serial commucication
  Serial.begin(115200);

  // Tries to start up LoRa module. If it doesn't start, goes into infinite loop
  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(freq)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // LoRa settings
  // These must match CanSat's settings!!!
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.enableCrc();
}

/*------------- Loop --------------*/
void loop() {
  // Check if new package has been received
  onReceive(LoRa.parsePacket());
}
