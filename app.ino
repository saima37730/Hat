/*
 * Offline Home Automation System - Master Controller (ESP32-S3)
 * 
 * This code implements a central control system that manages multiple slave devices
 * using ESP-NOW for communication. It provides a web dashboard via an access point
 * and uses various hardware components for local control and monitoring.
 * 
 * Hardware:
 * - ESP32-S3
 * - TM1638 Module (display, LEDs, buttons)
 * - DS3231 RTC Module
 * - SD Card Module
 * - Rotary Encoder
 * - 4-Channel Relay Module
 * 
 * Features:
 * - ESP-NOW communication with slaves
 * - Local and remote relay control
 * - Event logging with timestamps
 * - Persistent storage of states
 * - Web dashboard for monitoring and control
 * 
 * Created: Feb 2025
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <TM1638plus.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <AceButton.h>
#include <ESPAsyncWebServer.h>

using namespace ace_button;

// Pin Definitions
#define TM1638_STB 5   // Strobe pin for TM1638
#define TM1638_CLK 6   // Clock pin for TM1638
#define TM1638_DIO 7   // Data pin for TM1638
#define SD_CS 10       // CS pin for SD card
#define ROTARY_CLK 2   // Clock pin for rotary encoder
#define ROTARY_DT 3    // Data pin for rotary encoder
#define ROTARY_SW 4    // Switch pin for rotary encoder
#define RELAY1 12      // Relay 1 control pin
#define RELAY2 13      // Relay 2 control pin
#define RELAY3 14      // Relay 3 control pin
#define RELAY4 15      // Relay 4 control pin

// WiFi Credentials for Access Point
const char* ssid = "HomeAutomation";
const char* password = "12345678";
IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Initialize objects
TM1638plus tm(TM1638_STB, TM1638_CLK, TM1638_DIO);
RTC_DS3231 rtc;
AsyncWebServer server(80);
File dataFile;

// Rotary encoder variables
volatile int rotaryPosition = 0;
volatile boolean rotaryChanged = false;
int lastRotaryPosition = 0;
long lastActivityTime = 0;
const long inactivityTimeout = 10000; // 10 seconds

// Relay states
bool masterRelayStates[4] = {false, false, false, false};
const int relayPins[4] = {RELAY1, RELAY2, RELAY3, RELAY4};

// ESP-NOW variables
#define MAX_SLAVES 10
struct SlaveInfo {
  uint8_t macAddress[6];
  bool active;
  bool relayStates[4];
  String name;
};

SlaveInfo slaves[MAX_SLAVES];
int slaveCount = 0;

// Structure for sending commands
typedef struct {
  uint8_t targetRelay;  // 0-3
  bool state;           // true = on, false = off
} CommandData;

// Structure for receiving status updates
typedef struct {
  uint8_t relayStates[4];  // Array of relay states
} StatusData;

// Structure for schedules
typedef struct {
  int roomIndex;       // -1 = master, 0+ = slave index
  int relayIndex;      // 0-3
  bool targetState;    // true = on, false = off
  unsigned long targetTime;  // millis() when to execute
  bool executed;       // whether this has been executed
} ScheduleItem;

#define MAX_SCHEDULES 20
ScheduleItem schedules[MAX_SCHEDULES];
int scheduleCount = 0;

// Function declarations
void setupRTC();
void setupSD();
void setupRelays();
void setupRotaryEncoder();
void setupEspNow();
void setupWebServer();
void handleRotaryEncoder();
void updateDisplay();
void updateTM1638LEDs();
void handleTM1638Buttons();
void sendCommandToSlave(int slaveIndex, int relayIndex, bool state);
void logRelayChange(int roomIndex, int relayIndex, bool state);
void saveRelayStates();
void loadRelayStates();
void saveSlaveInfo();
void loadSlaveInfo();
void processSchedules();
String getMacString(const uint8_t* mac);
String getCurrentTimeString();
String getFormattedDuration(unsigned long seconds);
void handleWebRoot(AsyncWebServerRequest *request);
void handleRelayToggle(AsyncWebServerRequest *request);
void handleAddSlave(AsyncWebServerRequest *request);
void handleSchedule(AsyncWebServerRequest *request);
void handleGetStatus(AsyncWebServerRequest *request);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Master Controller...");
  
  // Initialize TM1638
  tm.displayBegin();
  tm.displayText("STARTING");
  
  // Setup components
  setupRTC();
  setupSD();
  setupRelays();
  setupRotaryEncoder();
  
  // Setup WiFi Access Point
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // Setup ESP-NOW
  setupEspNow();
  
  // Load saved data
  loadSlaveInfo();
  loadRelayStates();
  
  // Setup web server
  setupWebServer();
  
  lastActivityTime = millis();
  tm.displayText("ROOM   0");
}

void loop() {
  // Check rotary encoder
  handleRotaryEncoder();
  
  // Handle TM1638 buttons
  handleTM1638Buttons();
  
  // Process any scheduled actions
  processSchedules();
  
  // Update display if needed
  updateDisplay();
  
  delay(10);  // Small delay to prevent CPU hogging
}

// Setup RTC module
void setupRTC() {
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
  } else {
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, setting to compile time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    Serial.println("RTC initialized");
  }
}

// Setup SD card
void setupSD() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
  } else {
    Serial.println("SD Card initialized");
    
    // Create files if they don't exist
    if (!SD.exists("/relays.txt")) {
      dataFile = SD.open("/relays.txt", FILE_WRITE);
      dataFile.println("Timestamp,Room,Relay,State,Duration");
      dataFile.close();
    }
    
    if (!SD.exists("/states.txt")) {
      dataFile = SD.open("/states.txt", FILE_WRITE);
      dataFile.println("# Room,Relay,State");
      dataFile.close();
    }
    
    if (!SD.exists("/slaves.txt")) {
      dataFile = SD.open("/slaves.txt", FILE_WRITE);
      dataFile.println("# MAC,Name");
      dataFile.close();
    }
  }
}

// Setup relay pins
void setupRelays() {
  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);  // Relays are active LOW
  }
}

// Setup rotary encoder
void setupRotaryEncoder() {
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT, INPUT_PULLUP);
  pinMode(ROTARY_SW, INPUT_PULLUP);
  
  // Interrupt for rotary encoder
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), []() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();
    
    // Debounce
    if (interruptTime - lastInterruptTime > 5) {
      if (digitalRead(ROTARY_DT) == LOW) {
        rotaryPosition++;
      } else {
        rotaryPosition--;
      }
      
      // Limit to valid room range
      if (rotaryPosition < 0) rotaryPosition = 0;
      if (rotaryPosition > slaveCount) rotaryPosition = slaveCount;
      
      rotaryChanged = true;
      lastActivityTime = millis();
    }
    lastInterruptTime = interruptTime;
  }, FALLING);
}

// Setup ESP-NOW
void setupEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
  
  Serial.println("ESP-NOW initialized");
}

// Setup web server
void setupWebServer() {
  server.on("/", HTTP_GET, handleWebRoot);
  server.on("/toggle", HTTP_GET, handleRelayToggle);
  server.on("/add_slave", HTTP_POST, handleAddSlave);
  server.on("/schedule", HTTP_POST, handleSchedule);
  server.on("/status", HTTP_GET, handleGetStatus);
  
  server.begin();
  Serial.println("Web server started");
}

// Handle rotary encoder position changes
void handleRotaryEncoder() {
  if (rotaryChanged) {
    rotaryChanged = false;
    lastActivityTime = millis();
    
    // Update display with selected room
    char displayText[9];
    sprintf(displayText, "ROOM   %d", rotaryPosition);
    tm.displayText(displayText);
    
    // Update LEDs to show current relay states for selected room
    updateTM1638LEDs();
  }
}

// Update TM1638 display based on mode and inactivity timeout
void updateDisplay() {
  unsigned long currentTime = millis();
  
  // Show time after inactivity
  if (currentTime - lastActivityTime > inactivityTimeout) {
    DateTime now = rtc.now();
    char timeStr[9];
    sprintf(timeStr, "%02d%02d%02d  ", now.hour(), now.minute(), now.second());
    tm.displayText(timeStr);
  }
}

// Update TM1638 LEDs to show relay states of selected room
void updateTM1638LEDs() {
  uint8_t ledValues = 0;
  
  if (rotaryPosition == 0) {
    // Master room
    for (int i = 0; i < 4; i++) {
      if (masterRelayStates[i]) {
        ledValues |= (1 << i);
      }
    }
  } else if (rotaryPosition <= slaveCount) {
    // Slave room
    int slaveIndex = rotaryPosition - 1;
    for (int i = 0; i < 4; i++) {
      if (slaves[slaveIndex].relayStates[i]) {
        ledValues |= (1 << i);
      }
    }
  }
  
  tm.setLEDs(ledValues);
}

// Handle TM1638 button presses
void handleTM1638Buttons() {
  uint8_t buttons = tm.readButtons();
  
  if (buttons > 0) {
    lastActivityTime = millis();
    
    // Only handle the first 4 buttons (for relay control)
    for (int i = 0; i < 4; i++) {
      if (buttons & (1 << i)) {
        if (rotaryPosition == 0) {
          // Toggle master relay
          masterRelayStates[i] = !masterRelayStates[i];
          digitalWrite(relayPins[i], masterRelayStates[i] ? LOW : HIGH);  // Relays are active LOW
          logRelayChange(-1, i, masterRelayStates[i]);
          saveRelayStates();
        } else if (rotaryPosition <= slaveCount) {
          // Send command to slave
          int slaveIndex = rotaryPosition - 1;
          bool newState = !slaves[slaveIndex].relayStates[i];
          sendCommandToSlave(slaveIndex, i, newState);
        }
        
        // Update LEDs
        updateTM1638LEDs();
        
        // Small delay to prevent multiple toggles
        delay(200);
      }
    }
  }
}

// Send command to slave via ESP-NOW
void sendCommandToSlave(int slaveIndex, int relayIndex, bool state) {
  if (slaveIndex < 0 || slaveIndex >= slaveCount) return;
  
  CommandData commandData;
  commandData.targetRelay = relayIndex;
  commandData.state = state;
  
  esp_err_t result = esp_now_send(slaves[slaveIndex].macAddress, (uint8_t*)&commandData, sizeof(CommandData));
  
  if (result == ESP_OK) {
    Serial.println("Command sent successfully");
    // Optimistically update the state (will be corrected if slave responds with different state)
    slaves[slaveIndex].relayStates[relayIndex] = state;
    logRelayChange(slaveIndex, relayIndex, state);
    saveRelayStates();
  } else {
    Serial.println("Error sending command");
  }
}

// Log relay state change with timestamp
void logRelayChange(int roomIndex, int relayIndex, bool state) {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card failed to mount");
    return;
  }
  
  DateTime now = rtc.now();
  String timestamp = String(now.year()) + "-" + 
                    String(now.month()) + "-" + 
                    String(now.day()) + " " + 
                    String(now.hour()) + ":" + 
                    String(now.minute()) + ":" + 
                    String(now.second());
  
  String roomName = (roomIndex == -1) ? "Master" : "Slave" + String(roomIndex + 1);
  String stateStr = state ? "ON" : "OFF";
  
  // Read previous log to calculate duration
  String prevState = "";
  unsigned long duration = 0;
  
  dataFile = SD.open("/relays.txt", FILE_READ);
  if (dataFile) {
    String lastLine = "";
    while (dataFile.available()) {
      String line = dataFile.readStringUntil('\n');
      if (line.indexOf(roomName + "," + String(relayIndex)) > 0) {
        lastLine = line;
      }
    }
    dataFile.close();
    
    if (lastLine != "") {
      int lastComma = lastLine.lastIndexOf(',');
      if (lastComma > 0) {
        prevState = lastLine.substring(lastLine.indexOf(",", lastLine.indexOf(",") + 1) + 1, lastComma);
        if (prevState != stateStr) {
          // Calculate duration
          String lastTimeStr = lastLine.substring(0, lastLine.indexOf(','));
          DateTime lastTime(lastTimeStr.substring(0, 4).toInt(),
                           lastTimeStr.substring(5, 7).toInt(),
                           lastTimeStr.substring(8, 10).toInt(),
                           lastTimeStr.substring(11, 13).toInt(),
                           lastTimeStr.substring(14, 16).toInt(),
                           lastTimeStr.substring(17, 19).toInt());
          duration = now.unixtime() - lastTime.unixtime();
        }
      }
    }
  }
  
  // Write new log entry
  dataFile = SD.open("/relays.txt", FILE_APPEND);
  if (dataFile) {
    dataFile.print(timestamp);
    dataFile.print(",");
    dataFile.print(roomName);
    dataFile.print(",");
    dataFile.print(relayIndex);
    dataFile.print(",");
    dataFile.print(stateStr);
    dataFile.print(",");
    dataFile.println(duration);
    dataFile.close();
  }
}

// Save current relay states to SD card
void saveRelayStates() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card failed to mount");
    return;
  }
  
  // Remove old file and create new
  if (SD.exists("/states.txt")) {
    SD.remove("/states.txt");
  }
  
  dataFile = SD.open("/states.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("# Room,Relay,State");
    
    // Save master relays
    for (int i = 0; i < 4; i++) {
      dataFile.print("Master,");
      dataFile.print(i);
      dataFile.print(",");
      dataFile.println(masterRelayStates[i] ? "ON" : "OFF");
    }
    
    // Save slave relays
    for (int s = 0; s < slaveCount; s++) {
      for (int i = 0; i < 4; i++) {
        dataFile.print("Slave");
        dataFile.print(s + 1);
        dataFile.print(",");
        dataFile.print(i);
        dataFile.print(",");
        dataFile.println(slaves[s].relayStates[i] ? "ON" : "OFF");
      }
    }
    
    dataFile.close();
    Serial.println("Relay states saved");
  }
}

// Load relay states from SD card
void loadRelayStates() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card failed to mount");
    return;
  }
  
  dataFile = SD.open("/states.txt", FILE_READ);
  if (dataFile) {
    // Skip header
    dataFile.readStringUntil('\n');
    
    while (dataFile.available()) {
      String line = dataFile.readStringUntil('\n');
      if (line.startsWith("#") || line.length() < 5) continue;
      
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      
      if (firstComma > 0 && secondComma > firstComma) {
        String roomName = line.substring(0, firstComma);
        int relayIndex = line.substring(firstComma + 1, secondComma).toInt();
        String stateStr = line.substring(secondComma + 1);
        bool state = (stateStr == "ON");
        
        if (roomName == "Master" && relayIndex >= 0 && relayIndex < 4) {
          masterRelayStates[relayIndex] = state;
          digitalWrite(relayPins[relayIndex], state ? LOW : HIGH);  // Relays are active LOW
        } else if (roomName.startsWith("Slave")) {
          int slaveIndex = roomName.substring(5).toInt() - 1;
          if (slaveIndex >= 0 && slaveIndex < slaveCount && relayIndex >= 0 && relayIndex < 4) {
            slaves[slaveIndex].relayStates[relayIndex] = state;
          }
        }
      }
    }
    
    dataFile.close();
    Serial.println("Relay states loaded");
  }
}

// Save slave information to SD card
void saveSlaveInfo() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card failed to mount");
    return;
  }
  
  // Remove old file and create new
  if (SD.exists("/slaves.txt")) {
    SD.remove("/slaves.txt");
  }
  
  dataFile = SD.open("/slaves.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("# MAC,Name");
    
    for (int i = 0; i < slaveCount; i++) {
      dataFile.print(getMacString(slaves[i].macAddress));
      dataFile.print(",");
      dataFile.println(slaves[i].name);
    }
    
    dataFile.close();
    Serial.println("Slave info saved");
  }
}

// Load slave information from SD card
void loadSlaveInfo() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card failed to mount");
    return;
  }
  
  slaveCount = 0;
  
  dataFile = SD.open("/slaves.txt", FILE_READ);
  if (dataFile) {
    // Skip header
    dataFile.readStringUntil('\n');
    
    while (dataFile.available() && slaveCount < MAX_SLAVES) {
      String line = dataFile.readStringUntil('\n');
      if (line.startsWith("#") || line.length() < 17) continue;
      
      int comma = line.indexOf(',');
      if (comma > 0) {
        String macStr = line.substring(0, comma);
        String name = line.substring(comma + 1);
        
        // Parse MAC address
        int macBytes[6];
        if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x", 
                   &macBytes[0], &macBytes[1], &macBytes[2], 
                   &macBytes[3], &macBytes[4], &macBytes[5]) == 6) {
          
          for (int i = 0; i < 6; i++) {
            slaves[slaveCount].macAddress[i] = (uint8_t)macBytes[i];
          }
          
          slaves[slaveCount].name = name;
          slaves[slaveCount].active = false;
          for (int i = 0; i < 4; i++) {
            slaves[slaveCount].relayStates[i] = false;
          }
          
          // Register peer
          esp_now_peer_info_t peerInfo;
          memcpy(peerInfo.peer_addr, slaves[slaveCount].macAddress, 6);
          peerInfo.channel = 0;
          peerInfo.encrypt = false;
          
          if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            slaveCount++;
          }
        }
      }
    }
    
    dataFile.close();
    Serial.print("Loaded ");
    Serial.print(slaveCount);
    Serial.println(" slaves");
  }
}

// Process any scheduled relay actions
void processSchedules() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < scheduleCount; i++) {
    if (!schedules[i].executed && currentTime >= schedules[i].targetTime) {
      // Execute scheduled action
      if (schedules[i].roomIndex == -1) {
        // Master relay
        if (schedules[i].relayIndex >= 0 && schedules[i].relayIndex < 4) {
          masterRelayStates[schedules[i].relayIndex] = schedules[i].targetState;
          digitalWrite(relayPins[schedules[i].relayIndex], masterRelayStates[schedules[i].relayIndex] ? LOW : HIGH);
          logRelayChange(-1, schedules[i].relayIndex, masterRelayStates[schedules[i].relayIndex]);
          saveRelayStates();
        }
      } else {
        // Slave relay
        int slaveIndex = schedules[i].roomIndex;
        if (slaveIndex >= 0 && slaveIndex < slaveCount && 
            schedules[i].relayIndex >= 0 && schedules[i].relayIndex < 4) {
          sendCommandToSlave(slaveIndex, schedules[i].relayIndex, schedules[i].targetState);
        }
      }
      
      // Mark as executed
      schedules[i].executed = true;
      
      // Update LEDs if this was for the currently selected room
      if ((rotaryPosition == 0 && schedules[i].roomIndex == -1) || 
          (rotaryPosition == schedules[i].roomIndex + 1)) {
        updateTM1638LEDs();
      }
    }
  }
  
  // Clean up executed schedules
  int newCount = 0;
  for (int i = 0; i < scheduleCount; i++) {
    if (!schedules[i].executed || currentTime - schedules[i].targetTime < 60000) {
      // Keep non-executed or recently executed (for status display)
      if (i != newCount) {
        schedules[newCount] = schedules[i];
      }
      newCount++;
    }
  }
  scheduleCount = newCount;
}

// Helper function to convert MAC to string
String getMacString(const uint8_t* mac) {
  char macStr[18];
  sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x", 
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

// Get current time as formatted string
String getCurrentTimeString() {
  DateTime now = rtc.now();
  char timeStr[20];
  sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second());
  return String(timeStr);
}

// Format duration in seconds to readable string
String getFormattedDuration(unsigned long seconds) {
  if (seconds < 60) {
    return String(seconds) + " sec";
  } else if (seconds < 3600) {
    return String(seconds / 60) + " min " + String(seconds % 60) + " sec";
  } else {
    unsigned long hours = seconds / 3600;
    unsigned long mins = (seconds % 3600) / 60;
    return String(hours) + " hr " + String(mins) + " min";
  }
}

// Web server handler for root page
void handleWebRoot(AsyncWebServerRequest *request) {
  String html = "<!DOCTYPE html>\n";
  html += "<html><head><title>Home Automation</title>\n";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>\n";
  html += "<style>\n";
  html += "body { font-family: Arial; margin: 0; padding: 20px; background: #f5f5f5; }\n";
  html += ".card { background: white; border-radius: 8px; padding: 15px; margin-bottom: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }\n";
  html += "h1, h2 { color: #333; }\n";
  html += ".relay { display: flex; justify-content: space-between; align-items: center; padding: 10px; border-bottom: 1px solid #eee; }\n";
  html += ".relay:last-child { border-bottom: none; }\n";
  html += ".btn { background: #4CAF50; color: white; border: none; padding: 8px 12px; border-radius: 4px; cursor: pointer; }\n";
  html += ".btn-off { background: #f44336; }\n";
  html += "form { margin-top: 15px; }\n";
  html += "input, select { padding: 8px; margin-right: 10px; border: 1px solid #ddd; border-radius: 4px; }\n";
  html += ".room-title { background: #eee; padding: 8px; border-radius: 4px; margin-bottom: 10px; }\n";
  html += "</style>\n";
  html += "</head><body>\n";
  
  html += "<h1>Home Automation Dashboard</h1>\n";
  
  // Current time
  html += "<div class='card'>\n";
  html += "<h2>System Status</h2>\n";
  html += "<p>Current Time: " + getCurrentTimeString() + "</p>\n";
  html += "<p>Connected Slaves: " + String(slaveCount) + "</p>\n";
  html += "</div>\n";
  
  // Master Room Controls
  html += "<div class='card'>\n";
  html += "<div class='room-title'>Master Room</div>\n";
  
  for (int i = 0; i < 4; i++) {
    html += "<div class='relay'>\n";
    html += "<span>Relay " + String(i + 1) + "</span>\n";
    html += "<button class='btn " + String(masterRelayStates[i] ? "btn-off" : "") + "' onclick='location.href=\"/toggle?room=-1&relay=" + String(i) + "&state=" + String(masterRelayStates[i] ? 0 : 1) + "\"'>" + String(masterRelayStates[i] ? "ON" : "OFF") + "</button>\n";
    html += "</div>\n";
  }
  html += "</div>\n";
  
  // Slave Room Controls
  for (int s = 0; s < slaveCount; s++) {
    html += "<div class='card'>\n";
    html += "<div class='room-title'>Slave Room " + String(s + 1) + " (" + slaves[s].name + ")</div>\n";
    
    for (int i = 0; i < 4; i++) {
      html += "<div class='relay'>\n";
      html += "<span>Relay " + String(i + 1) + "</span>\n";
      html += "<button class='btn " + String(slaves[s].relayStates[i] ? "btn-off" : "") + "' onclick='location.href=\"/toggle?room=" + String(s) + "&relay=" + String(i) + "&state=" + String(slaves[s].relayStates[i] ? 0 : 1) + "\"'>" + String(slaves[s].relayStates[i] ? "ON" : "OFF") + "</button>\n";
      html += "</div>\n";
    }
    html += "</div>\n";
  }
  
  // Schedule Form
  html += "<div class='card'>\n";
  html += "<h2>Schedule Action</h2>\n";
  html += "<form action='/schedule' method='post'>\n";
  html += "<select name='room' required>\n";
  html += "<option value='-1'>Master Room</option>\n";
  
  for (int s = 0; s < slaveCount; s++) {
    html += "<option value='" + String(s) + "'>Slave " + String(s + 1) + " (" + slaves[s].name + ")</option>\n";
  }
  
  html += "</select>\n";
  html += "<select name='relay' required>\n";
  html += "<option value='0'>Relay 1</option>\n";
  html += "<option value='1'>Relay 2</option>\n";
  html += "<option value='2'>Relay 3</option>\n";
  html += "<option value='3'>Relay 4</option>\n";
  html += "</select>\n";
  html += "<select name='state' required>\n";
  html += "<option value='1'>ON</option>\n";
  html += "<option value='0'>OFF</option>\n";
  html += "</select>\n";
  html += "<input type='number' name='minutes' placeholder='Minutes from now' required min='1' max='1440'>\n";
  html += "<button type='submit' class='btn'>Schedule</button>\n";
  html += "</form>\n";
  html += "</div>\n";
  
  // Add Slave Form
  html += "<div class='card'>\n";
  html += "<h2>Add New Slave</h2>\n";
  html += "<form action='/add_slave' method='post'>\n";
  html += "<input type='text' name='mac' placeholder='MAC Address (XX:XX:XX:XX:XX:XX)' required pattern='^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$'>\n";
  html += "<input type='text' name='name' placeholder='Room Name' required>\n";
  html += "<button type='submit' class='btn'>Add Slave</button>\n";
  html += "</form>\n";
  html += "</div>\n";
  
  // Active Schedules
  if (scheduleCount > 0) {
    html += "<div class='card'>\n";
    html += "<h2>Active Schedules</h2>\n";
    
    for (int i = 0; i < scheduleCount; i++) {
      if (!schedules[i].executed) {
        String roomName = (schedules[i].roomIndex == -1) ? "Master" : "Slave " + String(schedules[i].roomIndex + 1);
        String relayName = "Relay " + String(schedules[i].relayIndex + 1);
        String actionName = schedules[i].targetState ? "ON" : "OFF";
        unsigned long timeLeft = (schedules[i].targetTime > millis()) ? (schedules[i].targetTime - millis()) / 1000 : 0;
        
        html += "<p>" + roomName + " - " + relayName + " - Turn " + actionName + " in " + getFormattedDuration(timeLeft) + "</p>\n";
      }
    }
    
    html += "</div>\n";
  }
  
  // Recent Logs
  html += "<div class='card'>\n";
  html += "<h2>Recent Logs</h2>\n";
  
  if (SD.begin(SD_CS)) {
    dataFile = SD.open("/relays.txt", FILE_READ);
    if (dataFile) {
      // Skip header
      dataFile.readStringUntil('\n');
      
      // Get last 10 lines
      String lastLines[10];
      int lineCount = 0;
      
      while (dataFile.available()) {
        String line = dataFile.readStringUntil('\n');
        if (line.length() > 0) {
          lastLines[lineCount % 10] = line;
          lineCount++;
        }
      }
      
      dataFile.close();
      
      // Display in reverse order (newest first)
      int start = (lineCount < 10) ? 0 : lineCount % 10;
      for (int i = 0; i < min(lineCount, 10); i++) {
        int index = (start + i) % 10;
        String line = lastLines[index];
        html += "<p>" + line + "</p>\n";
      }
    } else {
      html += "<p>No logs available</p>\n";
    }
  } else {
    html += "<p>SD Card not available</p>\n";
  }
  
  html += "</div>\n";
  
  // Auto-refresh
  html += "<script>\n";
  html += "setTimeout(function() { location.reload(); }, 30000);\n";  // Refresh every 30 seconds
  html += "</script>\n";
  
  html += "</body></html>";
  
  request->send(200, "text/html", html);
}

// Web server handler for relay toggle
void handleRelayToggle(AsyncWebServerRequest *request) {
  if (request->hasParam("room") && request->hasParam("relay") && request->hasParam("state")) {
    int roomIndex = request->getParam("room")->value().toInt();
    int relayIndex = request->getParam("relay")->value().toInt();
    bool state = request->getParam("state")->value().toInt() == 1;
    
    if (roomIndex == -1) {
      // Master room
      if (relayIndex >= 0 && relayIndex < 4) {
        masterRelayStates[relayIndex] = state;
        digitalWrite(relayPins[relayIndex], state ? LOW : HIGH);  // Relays are active LOW
        logRelayChange(-1, relayIndex, state);
        saveRelayStates();
      }
    } else {
      // Slave room
      if (roomIndex >= 0 && roomIndex < slaveCount && relayIndex >= 0 && relayIndex < 4) {
        sendCommandToSlave(roomIndex, relayIndex, state);
      }
    }
  }
  
  request->redirect("/");
}

// Web server handler for adding new slave
void handleAddSlave(AsyncWebServerRequest *request) {
  if (request->hasParam("mac", true) && request->hasParam("name", true) && slaveCount < MAX_SLAVES) {
    String macStr = request->getParam("mac", true)->value();
    String name = request->getParam("name", true)->value();
    
    // Parse MAC address
    int macBytes[6];
    if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x", 
               &macBytes[0], &macBytes[1], &macBytes[2], 
               &macBytes[3], &macBytes[4], &macBytes[5]) == 6) {
      
      // Create new slave entry
      for (int i = 0; i < 6; i++) {
        slaves[slaveCount].macAddress[i] = (uint8_t)macBytes[i];
      }
      
      slaves[slaveCount].name = name;
      slaves[slaveCount].active = false;
      for (int i = 0; i < 4; i++) {
        slaves[slaveCount].relayStates[i] = false;
      }
      
      // Register peer
      esp_now_peer_info_t peerInfo;
      memcpy(peerInfo.peer_addr, slaves[slaveCount].macAddress, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      
      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        slaveCount++;
        saveSlaveInfo();
      }
    }
  }
  
  request->redirect("/");
}

// Web server handler for scheduling action
void handleSchedule(AsyncWebServerRequest *request) {
  if (request->hasParam("room", true) && request->hasParam("relay", true) && 
      request->hasParam("state", true) && request->hasParam("minutes", true) && 
      scheduleCount < MAX_SCHEDULES) {
    
    int roomIndex = request->getParam("room", true)->value().toInt();
    int relayIndex = request->getParam("relay", true)->value().toInt();
    bool state = request->getParam("state", true)->value().toInt() == 1;
    int minutes = request->getParam("minutes", true)->value().toInt();
    
    if (minutes > 0 && minutes <= 1440) {  // Max 24 hours (1440 minutes)
      schedules[scheduleCount].roomIndex = roomIndex;
      schedules[scheduleCount].relayIndex = relayIndex;
      schedules[scheduleCount].targetState = state;
      schedules[scheduleCount].targetTime = millis() + (minutes * 60 * 1000);
      schedules[scheduleCount].executed = false;
      scheduleCount++;
    }
  }
  
  request->redirect("/");
}

// Web server handler for getting current status (JSON)
void handleGetStatus(AsyncWebServerRequest *request) {
  String json = "{\n";
  
  // System info
  json += "  \"time\": \"" + getCurrentTimeString() + "\",\n";
  json += "  \"slaveCount\": " + String(slaveCount) + ",\n";
  
  // Master relays
  json += "  \"master\": {\n";
  json += "    \"relays\": [";
  for (int i = 0; i < 4; i++) {
    json += masterRelayStates[i] ? "true" : "false";
    if (i < 3) json += ", ";
  }
  json += "]\n  },\n";
  
  // Slaves
  json += "  \"slaves\": [\n";
  for (int s = 0; s < slaveCount; s++) {
    json += "    {\n";
    json += "      \"name\": \"" + slaves[s].name + "\",\n";
    json += "      \"mac\": \"" + getMacString(slaves[s].macAddress) + "\",\n";
    json += "      \"active\": " + String(slaves[s].active ? "true" : "false") + ",\n";
    json += "      \"relays\": [";
    for (int i = 0; i < 4; i++) {
      json += slaves[s].relayStates[i] ? "true" : "false";
      if (i < 3) json += ", ";
    }
    json += "]\n    }";
    if (s < slaveCount - 1) json += ",";
    json += "\n";
  }
  json += "  ]\n";
  
  json += "}";
  
  request->send(200, "application/json", json);
}

// ESP-NOW callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// ESP-NOW callback when data is received
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  // Find which slave sent the data
  int slaveIndex = -1;
  for (int i = 0; i < slaveCount; i++) {
    if (memcmp(mac_addr, slaves[i].macAddress, 6) == 0) {
      slaveIndex = i;
      slaves[i].active = true;
      break;
    }
  }
  
  if (slaveIndex != -1) {
    // Process status update
    if (data_len == sizeof(StatusData)) {
      StatusData *statusData = (StatusData*)data;
      
      // Update relay states for this slave
      for (int i = 0; i < 4; i++) {
        bool newState = statusData->relayStates[i] == 1;
        
        // Only log if state changed
        if (slaves[slaveIndex].relayStates[i] != newState) {
          slaves[slaveIndex].relayStates[i] = newState;
          logRelayChange(slaveIndex, i, newState);
          saveRelayStates();
          
          // Update LEDs if this slave is currently selected
          if (rotaryPosition == slaveIndex + 1) {
            updateTM1638LEDs();
          }
        }
      }
    }
  } else {
    // Unknown slave, check if we should add it
    if (slaveCount < MAX_SLAVES) {
      // Auto-add new slave with default name
      for (int i = 0; i < 6; i++) {
        slaves[slaveCount].macAddress[i] = mac_addr[i];
      }
      
      slaves[slaveCount].name = "Auto " + getMacString(mac_addr);
      slaves[slaveCount].active = true;
      for (int i = 0; i < 4; i++) {
        slaves[slaveCount].relayStates[i] = false;
      }
      
      // Register peer
      esp_now_peer_info_t peerInfo;
      memcpy(peerInfo.peer_addr, slaves[slaveCount].macAddress, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      
      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        slaveCount++;
        saveSlaveInfo();
      }
    }
  }
}
