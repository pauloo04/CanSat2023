/*----------------------- Libaries ------------------ */
#include <SPI.h>
#include <LoRa.h>
#include <SD.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>

/*------------------------- LCD ------------------ */
// LCD pins
const int rs = 33, en = 34, d4 = 35, d5 = 36, d6 = 37, d7 = 39;

// LCD object
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Variables that are received
String hour, minute, second, longitude, latitude, altitude, speed;

/*-------------------------- LoRa --------------------- */
// LoRa pins
const int csPin = 24;
const int resetPin = 26;
const int irqPin = 25;

// Frquency for LoRa in Hz
const long freq = 4339E5; // 433.9 MHz

// Last received message time
unsigned long last_message_time = 0;
int seconds_since_last = 0;

int message_rssi;
float message_snr;

/* ----------------------- SD Card ------------------------ */
// File object
File csvFile;

// Name of created csv file
String file_name;

// Received message index
String message_index;

// CSV file header
// NAMES MUST BE IN THE SAME ORDER AS THEY ARE SENT FROM CANSAT
// EACH NAME MUST BE SEPERATED BY A COMMA(,) AND WITHOUT A SPACE BETWEEN
String header = "message_index,gps_hour,gps_minute,gps_second,gps_latitude,gps_longitude,gps_altitude,gps_speed,temp_scd30,pressure,humidity_scd30,rssi,snr";

/*------------------------ Timing ------------------------- */
unsigned long current_millis, previous_millis, previous_millis2;
unsigned long lcd_update_interval = 100;

/*----------------------- Functions ------------------------- */
// Create a new csv file and write the header with names of variables
void create_sd_file()
{
  // Get last index of created csv file
  int current_index = 0;
  EEPROM.get(0, current_index);

  // Increase index
  current_index += 1;

  // Creates the file name
  file_name = "BaseStationLog" + (String)current_index + ".csv";

  // If file with that index already exists, increment index until the index isn't taken
  while (SD.exists(file_name.c_str())) {
    current_index += 1;
    file_name = "BaseStationLog" + (String)current_index + ".csv";
  }

  // Creates a new file
  File csvFile = SD.open(file_name.c_str(), FILE_WRITE);

  // If the file is available, write to it:
  if (csvFile) {
    csvFile.println(header);
    csvFile.close();
  }

  // Store the new index, that just got taken
  EEPROM.put(0, current_index);
}

// Function that writes given message to SD card csv file
void write_to_sd(String message)
{
  // Opens a file
  File csvFile = SD.open(file_name.c_str(), FILE_WRITE);

  // if the file is available, write to it:
  if (csvFile) {
    csvFile.print(message);
    csvFile.close();
  }
}

// Function that runs waits until data package has been received
void onReceive(int packetSize) {
  // If no package received stop function
  if (packetSize == 0) return;

  // String to save incoming package
  String incoming = "";

  // Read package data and append each charachter to string
  while (LoRa.available()) {
    // Turn on LED to see that data has been received
    digitalWrite(14, HIGH);
    incoming += (char)LoRa.read();
  }

  message_rssi = LoRa.packetRssi();
  message_snr = LoRa.packetSnr();
  // At the end of the message append the signal strength and signal to noise ratio
  incoming += ",";
  incoming += message_rssi;
  incoming += ",";
  incoming += message_snr;
  incoming += "\n";
  
  // Print the message to serial monitor
  Serial.print(incoming);

  // Splits message to individual variables and stores them in an array 
  int MyP = 0;
  int MyI = 0;
  String array[12];
  int index = 0;
  while (MyI != -1) {
    MyI = incoming.indexOf(",", MyP);
    if (MyI == -1){
      break;
    }
    String s = incoming.substring(MyP, MyI);
    MyP = MyI + 1;
    array[index] = s;
    index = index + 1;
  }
  // Saves variables to indiidual strings
  message_index = array[0];
  hour = array[1];
  minute = array[2];
  second = array[3];
  latitude = array[4];
  longitude = array[5];
  altitude = array[6];
  speed = array[7];
  
  // Write data to sd card
  write_to_sd(incoming);
  
  last_message_time = millis();
  
  // Turn off LED as data has been parsed
  digitalWrite(14, LOW);
}

// Function that updates the LCD with new data
void update_lcd() {
  // Clear the display
  lcd.clear();
  // Update time since last message was reeceived
  seconds_since_last = (millis() - last_message_time) / 1000;
  
  // GPS
  lcd.setCursor(0, 0);
  lcd.print(hour);
  lcd.setCursor(2, 0);
  lcd.print(":");
  lcd.setCursor(3,0);
  lcd.print(minute);
  lcd.setCursor(5, 0);
  lcd.print(":");
  lcd.setCursor(6,0);
  lcd.print(second);
  lcd.setCursor(9, 0);
  lcd.print(seconds_since_last);
  lcd.setCursor(12, 0);
  lcd.print(message_index);
  
  // Coordinates
  lcd.setCursor(0, 1);
  lcd.print(latitude);
  lcd.setCursor(0, 2);
  lcd.print(longitude);
  
  // Altitude and speed
  lcd.setCursor(0, 3);
  lcd.print(altitude);
  lcd.setCursor(10, 3);
  lcd.print(speed);

  // Message info
  lcd.setCursor(12, 1);
  lcd.print(message_rssi);
  lcd.setCursor(12, 2);
  lcd.print(message_snr, 1);
}

/*------------- Setup --------------*/
void setup() {
  // Set pins to required mode
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);

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

  // See if the SD card is present and initialize it
  while (!SD.begin(BUILTIN_SDCARD)) {
    delay(1000);
  }
  // Create new csv file to store received data
  create_sd_file();

  // Begin LCD display
  lcd.begin(16, 4);
  lcd.clear();
  lcd.setCursor(0, 0);
}

/*------------- Loop --------------*/
void loop() {
  // Check if new package has been received
  onReceive(LoRa.parsePacket());

  // Get current millis
  current_millis = millis();

  if (current_millis - previous_millis2 >= 9500){
    lcd.clear();
    previous_millis2 = current_millis;
  }
  // If interval time has passed, update displayed data on LCD
  if (current_millis - previous_millis >= lcd_update_interval) {
    // Update displayed data
    update_lcd();

    // Update last time this update has run
    previous_millis = current_millis;
  }
  
}
