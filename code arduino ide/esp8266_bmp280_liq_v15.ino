/*******************************************************************************
 * PROJECT NAME:  ESP8266 Smart Environmental Station (Non-Blocking Offline Safe)
 * TARGET BOARD:  LOLIN(WEMOS) D1 mini (or compatible ESP8266 mini board)
 * FIRMWARE VERSION: v15-SixSigmaImproved-MQTT
 *            - Inherits all v14 features (I2C recovery, Night mode LCD, WiFiManager)
 *            - Non-blocking MQTT integration (PubSubClient)
 *            - Home Assistant Auto-Discovery support
 * HARDWARE WIRING COMPOSITION:
 *   [ESP8266 Mini Pin]   --->   [Peripherals & Bus Lines]
 *   5V (or VIN)          --->   VCC of 16x2 LCD AND VCC of 5V BMP280 Module
 *   GND                  --->   GND of 16x2 LCD, BMP280, and LED Anode (-)
 *   D2 (GPIO 4)          --->   SDA of 16x2 LCD AND SDA of BMP280 Module
 *   D1 (GPIO 5)          --->   SCL of 16x2 LCD AND SCL of BMP280 Module
 *   D3 (GPIO 0)          --->   Wi-Fi Status LED (+ Anode via 220 Ohm Resistor)
 ******************************************************************************/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>
#include <time.h>                   // Core C Time library for POSIX DST management

// ESP8266 Core Network Implementations
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>

// MQTT Implementation
#include <PubSubClient.h>

// =============================================================================
// GLOBAL CONFIGURATION AND SYSTEM STATE DEFINITIONS
// =============================================================================
const String CONFIG_VERSION   = "v15-SixSigmaImproved-MQTT";
const char*  DNS_LOCAL_NAME   = "weer";     // Resolves to http://weer.local

// MQTT Broker Configuration (Change these to match your setup)
const char*  MQTT_SERVER      = "192.168.1.100"; // <--- IP ADRES VAN JE MQTT BROKER / HOME ASSISTANT
const int    MQTT_PORT        = 1883;
const char*  MQTT_USER        = "";              // Blank if no auth
const char*  MQTT_PASS        = "";              // Blank if no auth
const char*  MQTT_DEVICE_ID   = "esp8266_weerstation";
const char*  MQTT_DEVICE_NAME = "ESP8266 Weerstation";

// Hardware Addressing
const int    LCD_I2C_ADDRESS  = 0x27;       // Alternate fallback: 0x3F
const int    BMP_I2C_ADDRESS  = 0x76;       // Alternate fallback: 0x77

// Display Configuration
const int    LCD_COLUMNS      = 16;
const int    LCD_ROWS         = 2;

// Pin Definitions
const int    I2C_SDA_PIN      = D2;         // GPIO 4
const int    I2C_SCL_PIN      = D1;         // GPIO 5
const int    WIFI_LED_PIN     = D3;         // GPIO 0 - Wi-Fi status indicator LED

// POSIX Timezone Definition for Belgium (Europe/Brussels)
const char*  TZ_BELGIUM       = "CET-1CEST,M3.5.0,M10.5.0/3";
const char*  NTP_SERVER_1     = "be.pool.ntp.org";
const char*  NTP_SERVER_2     = "europe.pool.ntp.org";
const char*  NTP_SERVER_3     = "time.nist.gov";

// Configuration Features
bool ENABLE_NIGHT_MODE        = false;  // Boolean to toggle LCD night mode feature ON/OFF
bool isBacklightOff           = false; // Internal tracking state for LCD backlight

// Shared Data Buffers
float globalTemperature = 0.0;
float globalPressure    = 0.0;
bool  isNetworkOnline   = false;

// Differential rendering cache (To prevent redraw flickering)
char lastLCDTempStr[10] = "";
char lastLCDTimeStr[10] = "";
char lastLCDDateStr[20] = "";

// Global Object Driver Instances
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);
Adafruit_BMP280   bmp;
ESP8266WebServer  server(80);
WiFiManager       wifiManager;

WiFiClient        espClient;
PubSubClient      mqttClient(espClient);

// Non-blocking loop timing tracking registers
unsigned long lastSensorUpdateTimestamp = 0;
const long    SENSOR_UPDATE_INTERVAL    = 3000;  // Read BMP280 every 3 seconds

unsigned long lastTimeUpdateTimestamp   = 0;
const long    TIME_UPDATE_INTERVAL      = 2000;  // Check LCD diff every 2000 ms

unsigned long lastWiFiCheckTimestamp    = 0;
// Dynamic Wi-Fi Interval (Six Sigma adjustment): 60s when online, 10s when reconnecting
const long    WIFI_CHECK_INTERVAL_ONLINE  = 60000; 
const long    WIFI_CHECK_INTERVAL_OFFLINE = 10000; 

// MQTT Reconnect Timing
unsigned long lastMqttReconnectTimestamp = 0;
const long    MQTT_RECONNECT_INTERVAL    = 30000; // Try MQTT reconnect every 30s if offline

// =============================================================================
// I2C BUS RECOVERY & RECONNECT ROUTINE
// =============================================================================

void recoverI2CBus() {
  Serial.println("[I2C-BUS] Executing bus recovery sequence...");
  Wire.pins(I2C_SDA_PIN, I2C_SCL_PIN);
  
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(10);

  // Clock SCL line 9 times to flush out stuck slave state machines
  for (int i = 0; i < 9; i++) {
    pinMode(I2C_SCL_PIN, OUTPUT);
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(10);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(10);
  }

  // Generate I2C STOP condition
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(10);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(10);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  delayMicroseconds(10);

  // Re-initialize Wire Hardware Subsystem
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(100);
  
  lcd.init();
  if (!isBacklightOff) {
    lcd.backlight();
  } else {
    lcd.noBacklight();
  }
  
  bmp.begin(BMP_I2C_ADDRESS);
  Serial.println("[I2C-BUS] Recovery completed. Drivers re-initialized.");
}

// =============================================================================
// MQTT & HOME ASSISTANT AUTO-DISCOVERY ROUTINES
// =============================================================================

void publishHomeAssistantDiscovery() {
  if (!mqttClient.connected()) return;

  Serial.println("[MQTT] Publishing Home Assistant Auto-Discovery configs...");

  // 1. Temperature Sensor
  String tempTopic = "homeassistant/sensor/" + String(MQTT_DEVICE_ID) + "_temp/config";
  String tempPayload = "{"
    "\"name\":\"Temperatuur\","
    "\"stat_t\":\"" + String(MQTT_DEVICE_ID) + "/state\","
    "\"unit_of_meas\":\"°C\","
    "\"dev_cla\":\"temperature\","
    "\"val_tpl\":\"{{ value_json.temperature }}\","
    "\"uniq_id\":\"" + String(MQTT_DEVICE_ID) + "_temp\","
    "\"dev\":{\"ids\":[\"" + String(MQTT_DEVICE_ID) + "\"],\"name\":\"" + String(MQTT_DEVICE_NAME) + "\",\"mdl\":\"ESP8266\",\"mf\":\"DIY\"}"
  "}";
  mqttClient.publish(tempTopic.c_str(), tempPayload.c_str(), true);

  // 2. Barometric Pressure Sensor
  String pressTopic = "homeassistant/sensor/" + String(MQTT_DEVICE_ID) + "_press/config";
  String pressPayload = "{"
    "\"name\":\"Luchtdruk\","
    "\"stat_t\":\"" + String(MQTT_DEVICE_ID) + "/state\","
    "\"unit_of_meas\":\"hPa\","
    "\"dev_cla\":\"pressure\","
    "\"val_tpl\":\"{{ value_json.pressure }}\","
    "\"uniq_id\":\"" + String(MQTT_DEVICE_ID) + "_press\","
    "\"dev\":{\"ids\":[\"" + String(MQTT_DEVICE_ID) + "\"],\"name\":\"" + String(MQTT_DEVICE_NAME) + "\"}"
  "}";
  mqttClient.publish(pressTopic.c_str(), pressPayload.c_str(), true);

  // 3. Wi-Fi Signal Strength
  String rssiTopic = "homeassistant/sensor/" + String(MQTT_DEVICE_ID) + "_rssi/config";
  String rssiPayload = "{"
    "\"name\":\"Wi-Fi Signaal\","
    "\"stat_t\":\"" + String(MQTT_DEVICE_ID) + "/state\","
    "\"unit_of_meas\":\"dBm\","
    "\"dev_cla\":\"signal_strength\","
    "\"val_tpl\":\"{{ value_json.rssi }}\","
    "\"uniq_id\":\"" + String(MQTT_DEVICE_ID) + "_rssi\","
    "\"entity_category\":\"diagnostic\","
    "\"dev\":{\"ids\":[\"" + String(MQTT_DEVICE_ID) + "\"],\"name\":\"" + String(MQTT_DEVICE_NAME) + "\"}"
  "}";
  mqttClient.publish(rssiTopic.c_str(), rssiPayload.c_str(), true);
}

bool reconnectMQTT() {
  if (!isNetworkOnline) return false;

  Serial.print("[MQTT] Attempting connection to ");
  Serial.print(MQTT_SERVER);
  Serial.print("...");

  String clientId = String(MQTT_DEVICE_ID) + "-" + String(ESP.getChipId(), HEX);
  
  bool connected = false;
  if (strlen(MQTT_USER) > 0) {
    connected = mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);
  } else {
    connected = mqttClient.connect(clientId.c_str());
  }

  if (connected) {
    Serial.println(" CONNECTED!");
    publishHomeAssistantDiscovery();
  } else {
    Serial.print(" FAILED, rc=");
    Serial.println(mqttClient.state());
  }
  return connected;
}

void sendMqttTelemetry() {
  if (!mqttClient.connected()) return;

  String payload = "{";
  payload += "\"temperature\":" + String(globalTemperature, 1) + ",";
  payload += "\"pressure\":" + String(globalPressure, 1) + ",";
  payload += "\"rssi\":" + String(WiFi.RSSI());
  payload += "}";

  String stateTopic = String(MQTT_DEVICE_ID) + "/state";
  mqttClient.publish(stateTopic.c_str(), payload.c_str());
  Serial.println("[MQTT] Telemetry published successfully.");
}

// =============================================================================
// TIMEKEEPING & NTP ROUTINES
// =============================================================================

void configureTimeServices() {
  Serial.println("[TIME-SYSTEM] Initializing Belgian Time Zone POSIX parameters...");
  configTime(TZ_BELGIUM, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
}

bool getFormattedLocalTime(char* timeBuffer, char* dateBuffer) {
  time_t now = time(nullptr);
  
  if (now < 1577836800) { // Prior to 2020 -> Not yet synchronized
    sprintf(timeBuffer, "--:--");
    sprintf(dateBuffer, "--- --/--/--");
    return false;
  }

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  strftime(timeBuffer, 6, "%H:%M", &timeinfo);         // 00:25
  strftime(dateBuffer, 14, "%a %d/%m/%y", &timeinfo);  // Mon 28/09/26
  return true;
}

void updateBacklightSchedule() {
  if (!ENABLE_NIGHT_MODE) {
    if (isBacklightOff) {
      lcd.backlight();
      isBacklightOff = false;
    }
    return;
  }

  time_t now = time(nullptr);
  if (now < 1577836800) return; // Time not set yet

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  int currentHour = timeinfo.tm_hour;

  // Night hours active from 01:00 up to 04:59 (1 <= hour < 5)
  if (currentHour >= 1 && currentHour < 5) {
    if (!isBacklightOff) {
      lcd.noBacklight();
      isBacklightOff = true;
      Serial.println("[LCD] Night mode active: Backlight turned OFF (01:00 - 05:00)");
    }
  } else {
    if (isBacklightOff) {
      lcd.backlight();
      isBacklightOff = false;
      Serial.println("[LCD] Day mode active: Backlight turned ON");
    }
  }
}

// =============================================================================
// SERIAL DIAGNOSTIC PRINTER
// =============================================================================

void printSerialTelemetryHeader(const char* timeStr, const char* dateStr) {
  Serial.println("========================================");
  Serial.println("           TELEMETRY REPORT             ");
  Serial.println("========================================");
  Serial.print("Time    : "); Serial.println(timeStr);
  Serial.print("Date    : "); Serial.println(dateStr);
  Serial.print("Temp    : "); Serial.print(globalTemperature, 1); Serial.println(" \xC2\xB0""C");
  Serial.print("Pressure: "); Serial.print(globalPressure, 1); Serial.println(" hPa");
  
  if (isNetworkOnline) {
    Serial.println("Status  : ONLINE");
    Serial.print("SSID    : "); Serial.println(WiFi.SSID());
    Serial.print("IP Addr : "); Serial.println(WiFi.localIP());
    Serial.print("DNS Name: http://"); Serial.print(DNS_LOCAL_NAME); Serial.println(".local");
    Serial.print("MQTT    : "); Serial.println(mqttClient.connected() ? "CONNECTED" : "DISCONNECTED");
  } else {
    String targetSSID = WiFi.SSID();
    
    if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
      Serial.println("Status  : OFFLINE (AP Config Portal Active)");
      Serial.println("AP SSID : WeatherNodeAP");
    } else {
      Serial.println("Status  : OFFLINE");
    }

    if (targetSSID.length() > 0) {
      Serial.print("Connecting to: "); Serial.println(targetSSID);
    } else {
      Serial.println("Target SSID  : None chosen yet");
    }
  }
  Serial.println("========================================\n");
}

// =============================================================================
// WEB INTERFACE GENERATOR
// =============================================================================

void handleRootWebPage() {
  Serial.println("[WEB-SERVER] HTTP GET request parsed on root '/' route.");
  
  char timeBuf[6]  = "--:--";
  char dateBuf[14] = "--- --/--/--";
  getFormattedLocalTime(timeBuf, dateBuf);

  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Weather Station v15</title>";
  html += "<style>body{font-family:Arial,sans-serif; background:#f4f6f9; text-align:center; margin:0; padding:20px; color:#333;}";
  html += ".card{background:white; max-width:420px; margin:20px auto; padding:20px; border-radius:15px; box-shadow:0 4px 15px rgba(0,0,0,0.1);}";
  html += "h1{color:#2c3e50; font-size:22px;} .metric{font-size:28px; font-weight:bold; color:#27ae60; margin:10px 0;}";
  html += ".time-box{background:#eef2f7; padding:10px; border-radius:8px; font-size:20px; margin-bottom:15px; font-weight:bold;}";
  html += ".net-box{background:#f8f9fa; border:1px solid #e9ecef; border-radius:8px; padding:10px; font-size:13px; text-align:left; margin-top:15px;}";
  html += ".unit{font-size:16px; color:#7f8c8d;} .footer{font-size:11px; color:#bdc3c7; margin-top:20px;}</style>";
  html += "<script>setTimeout(function(){location.reload();}, 3000);</script>"; 
  html += "</head><body>";
  html += "<div class='card'><h1>Smart Weather Station</h1>";
  html += "<div class='time-box'>" + String(dateBuf) + " | " + String(timeBuf) + "</div>";
  html += "<hr style='border:0; border-top:1px solid #eee;'>";
  html += "<p>Temperature</p><div class='metric'>" + String(globalTemperature, 1) + "<span class='unit'> &deg;C</span></div>";
  html += "<p>Barometric Pressure</p><div class='metric'>" + String(globalPressure, 1) + "<span class='unit'> hPa</span></div>";
  
  html += "<div class='net-box'>";
  html += "<b>Network Status:</b> " + String(isNetworkOnline ? "Online" : "Offline") + "<br>";
  html += "<b>SSID:</b> " + WiFi.SSID() + "<br>";
  html += "<b>IP Address:</b> " + WiFi.localIP().toString() + "<br>";
  html += "<b>Local Hostname:</b> http://" + String(DNS_LOCAL_NAME) + ".local<br>";
  html += "<b>MQTT Status:</b> " + String(mqttClient.connected() ? "Connected" : "Disconnected") + "<br>";
  html += "<b>Night Mode Enabled:</b> " + String(ENABLE_NIGHT_MODE ? "Yes" : "No");
  html += "</div>";

  html += "<div class='footer'>Node: ESP8266 | Build: " + CONFIG_VERSION + "</div>";
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

// =============================================================================
// SUB-ROUTINE SYSTEM MODULES
// =============================================================================

void initializeSerialDiagnostics() {
  Serial.begin(115200);
  delay(1000); 
  Serial.flush();
  delay(500);
  Serial.println("\n========================================================");
  Serial.print("   SYSTEM BOOT UP: VERSION "); Serial.println(CONFIG_VERSION);
  Serial.println("   Target Platform: ESP8266 (LOLIN/WEMOS D1 Mini)");
  Serial.println("========================================================");
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("[WIFI-PORTAL] Manual Configuration Portal Triggered.");
  digitalWrite(WIFI_LED_PIN, LOW);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AP MODE ACTIVE  ");
  lcd.setCursor(0, 1);
  lcd.print("SSID:WeatherNode");
}

void initializeNetworkServices() {
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setAPCallback(configModeCallback);
  
  Serial.print("[NET-STAGE 1] Activating asynchronous Wi-Fi stack... ");
  wifiManager.autoConnect("WeatherNodeAP");
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("STARTING SYSTEM ");
  lcd.setCursor(0, 1);
  lcd.print("Checking WiFi...");
  
  delay(1000); 
  lcd.clear();
  lastLCDTempStr[0] = '\0';
  lastLCDTimeStr[0] = '\0';
  lastLCDDateStr[0] = '\0';
}

void checkNetworkStatus() {
  wifiManager.process();

  bool currentStatus = (WiFi.status() == WL_CONNECTED);

  if (currentStatus && !isNetworkOnline) {
    isNetworkOnline = true;
    digitalWrite(WIFI_LED_PIN, HIGH);
    
    Serial.println("\n[NET-STATUS] Network came ONLINE!");
    Serial.print("[NET-STATUS] Connected SSID: "); Serial.println(WiFi.SSID());
    Serial.print("[NET-STATUS] Local IP     : "); Serial.println(WiFi.localIP());
    
    configureTimeServices();

    if (MDNS.begin(DNS_LOCAL_NAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.print("[MDNS] Domain activated: http://");
      Serial.print(DNS_LOCAL_NAME);
      Serial.println(".local");
    }
    server.on("/", handleRootWebPage);
    server.begin();

    // Setup MQTT Server configuration
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    reconnectMQTT();
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WIFI CONNECTED! ");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP().toString());
    
    delay(1500);
    lcd.clear();
    lastLCDTempStr[0] = '\0';
    lastLCDTimeStr[0] = '\0';
    lastLCDDateStr[0] = '\0';
  } 
  else if (!currentStatus && isNetworkOnline) {
    isNetworkOnline = false;
    digitalWrite(WIFI_LED_PIN, LOW);
    Serial.println("[NET-STATUS] Network went OFFLINE!");
    server.stop();
  }
}

void initializeHardware() {
  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);

  lcd.init();          
  lcd.backlight(); 
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BOOTING DEVICE..");
  
  lcd.setCursor(0, 1);
  lcd.print("Init I2C Bus... ");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(400);
  lcd.print("OK");
  delay(400);
  
  Serial.print("[HW-STAGE 2] Probing BMP280 Sensor... ");
  lcd.setCursor(0, 1);
  lcd.print("Init BMP280...  ");
  
  if (!bmp.begin(BMP_I2C_ADDRESS)) {
    Serial.println("HARDWARE FAULT DETECTED! Running I2C recovery...");
    recoverI2CBus(); // Attempt unsticking bus before reporting error
    
    if (!bmp.begin(BMP_I2C_ADDRESS)) {
      Serial.println("CRITICAL HARDWARE FAULT UNRESOLVED!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("I2C BUS FAULT!  ");
      lcd.setCursor(0, 1);
      lcd.print("BMP280 DISCONN! ");
      
      while (1) {
        digitalWrite(WIFI_LED_PIN, !digitalRead(WIFI_LED_PIN));
        delay(250);
        ESP.wdtFeed(); 
        yield(); 
      }
    }
  }
  Serial.println("OK");
  lcd.print("OK");
  delay(800);
  lcd.clear();
}

void readSensorTelemetry() {
  float temp = bmp.readTemperature();
  float pres = bmp.readPressure() / 100.0F;

  if (!isnan(temp) && !isnan(pres) && temp >= -30.0 && temp <= 75.0) {
    globalTemperature = temp;
    globalPressure    = pres;

    // Send sensor values to MQTT if online
    sendMqttTelemetry();
  } else {
    Serial.println("[ERROR] Failed reading BMP280 sensor values! Triggering I2C recovery...");
    recoverI2CBus(); // Re-establish I2C bus upon consecutive read faults
  }
}

void renderLCDDisplay() {
  char currentTempStr[10];
  char currentTimeStr[10];
  char currentDateStr[20];

  dtostrf(globalTemperature, 4, 1, currentTempStr);
  getFormattedLocalTime(currentTimeStr, currentDateStr);

  // --- LINE 0: [Temp] [Time] ---
  char tempFormatted[8];
  sprintf(tempFormatted, "%s%cC", currentTempStr, (char)223);

  if (strcmp(tempFormatted, lastLCDTempStr) != 0) {
    lcd.setCursor(0, 0);
    lcd.print(tempFormatted);
    if (strlen(tempFormatted) < strlen(lastLCDTempStr)) {
      lcd.print(" ");
    }
    strcpy(lastLCDTempStr, tempFormatted);
  }

  if (strcmp(currentTimeStr, lastLCDTimeStr) != 0) {
    lcd.setCursor(11, 0);
    lcd.print(currentTimeStr);
    strcpy(lastLCDTimeStr, currentTimeStr);
  }

  // --- LINE 1: Date ---
  char datePadded[17];
  sprintf(datePadded, "%-16s", currentDateStr);

  if (strcmp(datePadded, lastLCDDateStr) != 0) {
    lcd.setCursor(0, 1);
    lcd.print(datePadded);
    strcpy(lastLCDDateStr, datePadded);
  }
}

// =============================================================================
// MAIN EXECUTION ROUTINES
// =============================================================================

void setup() {
  ESP.wdtFeed();
  
  initializeSerialDiagnostics();
  initializeHardware();
  initializeNetworkServices();
  configureTimeServices();

  readSensorTelemetry();
  renderLCDDisplay();
  Serial.println("[SYSTEM STATUS] Boot sequence complete. Processing main loop.\n");
}

void loop() {
  ESP.wdtFeed();

  wifiManager.process();
  yield(); 

  if (isNetworkOnline) {
    server.handleClient();
    MDNS.update();

    // MQTT Non-Blocking Keep-Alive & Reconnect Strategy
    if (!mqttClient.connected()) {
      unsigned long now = millis();
      if (now - lastMqttReconnectTimestamp >= MQTT_RECONNECT_INTERVAL) {
        lastMqttReconnectTimestamp = now;
        reconnectMQTT();
      }
    } else {
      mqttClient.loop();
    }
  }

  unsigned long currentMillis = millis();

  // 1. Asynchronous dynamic Wi-Fi check (Six Sigma: 60s online, 10s offline)
  long activeWiFiInterval = isNetworkOnline ? WIFI_CHECK_INTERVAL_ONLINE : WIFI_CHECK_INTERVAL_OFFLINE;
  if (currentMillis - lastWiFiCheckTimestamp >= activeWiFiInterval) {
    lastWiFiCheckTimestamp = currentMillis;
    checkNetworkStatus();
  }

  // 2. Read BMP280 Sensor telemetry, publish MQTT, update backlight schedule every 3 seconds
  if (currentMillis - lastSensorUpdateTimestamp >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdateTimestamp = currentMillis;
    readSensorTelemetry();
    updateBacklightSchedule();

    char currentTimeStr[10];
    char currentDateStr[20];
    getFormattedLocalTime(currentTimeStr, currentDateStr);

    printSerialTelemetryHeader(currentTimeStr, currentDateStr);
  }

  // 3. Differential LCD render check every 2000 ms
  if (currentMillis - lastTimeUpdateTimestamp >= TIME_UPDATE_INTERVAL) {
    lastTimeUpdateTimestamp = currentMillis;
    renderLCDDisplay();
  }
}                                                                                      