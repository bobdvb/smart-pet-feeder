/*
 * Smart Cat Feeder - ESP32 Controller
 *
 * Features:
 * - 5-slot rotating carousel with motor control
 * - Photocell position sensing with calibration
 * - Scheduled feeding times (2x per day)
 * - TEC temperature control with PID
 * - Proportional fan speed control
 * - DS18B20 temperature sensor
 * - WiFiManager configuration portal
 * - NTP time synchronization
 * - Web-based control interface
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define MOTOR_PWM_PIN      25    // PWM output for motor speed
#define MOTOR_DIR_PIN      26    // Motor direction control
#define PHOTOCELL_PIN      27    // Photocell sensor input (digital)
#define TEMP_SENSOR_PIN    14    // DS18B20 1-Wire temperature sensor
#define TEC_PWM_PIN        32    // PWM output for TEC cooler
#define FAN_PWM_PIN        33    // PWM output for fan

// ============================================================================
// CONSTANTS
// ============================================================================
#define SLOTS_COUNT        5     // Number of carousel slots
#define MOTOR_TIMEOUT      30000 // Motor timeout in milliseconds
#define MOTOR_SPEED        255   // Motor PWM speed (0-255)
#define TEMP_CHECK_INTERVAL 5000 // Temperature check interval (ms)
#define TIME_CHECK_INTERVAL 1000 // Time check interval (ms)
#define NTP_UPDATE_INTERVAL 3600000 // NTP update interval (1 hour)

// PWM Configuration
#define PWM_FREQ          5000   // PWM frequency in Hz
#define PWM_RESOLUTION    8      // PWM resolution in bits (0-255)
#define MOTOR_PWM_CHANNEL 0
#define TEC_PWM_CHANNEL   1
#define FAN_PWM_CHANNEL   2

// Temperature Control
#define MIN_TEMP          5.0    // Minimum temperature (°C)
#define MAX_TEMP          30.0   // Maximum temperature (°C)
#define FAN_MIN_SPEED     50     // Minimum fan speed (0-255)
#define FAN_MAX_SPEED     255    // Maximum fan speed (0-255)

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
Preferences preferences;
WebServer server(80);
WiFiManager wifiManager;
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// ============================================================================
// CONFIGURATION STRUCTURE
// ============================================================================
struct Config {
  char wifiSSID[32];
  char wifiPassword[64];
  int feedTime1Hour;         // First feeding time (hour)
  int feedTime1Minute;       // First feeding time (minute)
  int feedTime2Hour;         // Second feeding time (hour)
  int feedTime2Minute;       // Second feeding time (minute)
  float targetTemp;          // Target temperature for TEC
  int timezoneOffset;        // Timezone offset in hours
  int calibrationOffset;     // Calibration offset in milliseconds
  char ntpServer[64];        // NTP server address
  bool enableCooling;        // Enable/disable TEC cooling

  // Default values
  Config() {
    strcpy(wifiSSID, "");
    strcpy(wifiPassword, "");
    feedTime1Hour = 7;
    feedTime1Minute = 0;
    feedTime2Hour = 18;
    feedTime2Minute = 0;
    targetTemp = 15.0;
    timezoneOffset = 0;
    calibrationOffset = 0;
    strcpy(ntpServer, "pool.ntp.org");
    enableCooling = true;
  }
};

Config config;

// ============================================================================
// STATE VARIABLES
// ============================================================================
int currentSlot = 0;
bool feedingInProgress = false;
bool motorRunning = false;
unsigned long motorStartTime = 0;
unsigned long lastTempCheck = 0;
unsigned long lastTimeCheck = 0;
unsigned long lastNTPUpdate = 0;
bool lastFeedTime1Done = false;
bool lastFeedTime2Done = false;
float currentTemp = 0.0;
float pidIntegral = 0.0;
float pidLastError = 0.0;
bool photocellTriggered = false;
bool waitingForEdge = false;

// PID parameters for temperature control
const float Kp = 2.0;
const float Ki = 0.1;
const float Kd = 1.0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void setupPins();
void setupWiFi();
void setupWebServer();
void setupNTP();
void loadConfig();
void saveConfig();
void rotateCarousel();
void checkFeedingSchedule();
void updateTemperatureControl();
void readTemperature();
int calculateFanSpeed(float temp);
int calculateTECPower(float temp);
void handleRoot();
void handleConfig();
void handleSaveConfig();
void handleManualFeed();
void handleStatus();
void handleNotFound();
String getHTMLPage();
String getStatusJSON();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\nSmart Cat Feeder - Starting...");

  // Initialize pins
  setupPins();

  // Load configuration from preferences
  loadConfig();

  // Initialize temperature sensor
  tempSensor.begin();
  readTemperature();

  // Setup WiFi with WiFiManager
  setupWiFi();

  // Setup NTP time synchronization
  setupNTP();

  // Setup web server
  setupWebServer();

  Serial.println("Setup complete!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  server.handleClient();

  unsigned long currentMillis = millis();

  // Check feeding schedule every second
  if (currentMillis - lastTimeCheck >= TIME_CHECK_INTERVAL) {
    lastTimeCheck = currentMillis;
    checkFeedingSchedule();
  }

  // Update temperature control every 5 seconds
  if (currentMillis - lastTempCheck >= TEMP_CHECK_INTERVAL) {
    lastTempCheck = currentMillis;
    readTemperature();
    updateTemperatureControl();
  }

  // Update NTP time every hour
  if (currentMillis - lastNTPUpdate >= NTP_UPDATE_INTERVAL) {
    lastNTPUpdate = currentMillis;
    setupNTP();
  }

  // Monitor motor timeout
  if (motorRunning) {
    if (currentMillis - motorStartTime > MOTOR_TIMEOUT) {
      Serial.println("Motor timeout - stopping");
      stopMotor();
      feedingInProgress = false;
      waitingForEdge = false;
    }
  }
}

// ============================================================================
// PIN SETUP
// ============================================================================
void setupPins() {
  // Motor control
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);

  // Configure PWM channels
  ledcSetup(MOTOR_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PWM_PIN, MOTOR_PWM_CHANNEL);
  ledcWrite(MOTOR_PWM_CHANNEL, 0);

  ledcSetup(TEC_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(TEC_PWM_PIN, TEC_PWM_CHANNEL);
  ledcWrite(TEC_PWM_CHANNEL, 0);

  ledcSetup(FAN_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0);

  // Photocell sensor
  pinMode(PHOTOCELL_PIN, INPUT_PULLUP);

  Serial.println("Pins configured");
}

// ============================================================================
// WIFI SETUP
// ============================================================================
void setupWiFi() {
  Serial.println("Setting up WiFi...");

  // Set custom AP name
  wifiManager.setConfigPortalTimeout(180);

  // Add custom parameters for configuration
  WiFiManagerParameter custom_feed1_hour("feed1h", "Morning Feed Hour (0-23)", String(config.feedTime1Hour).c_str(), 3);
  WiFiManagerParameter custom_feed1_min("feed1m", "Morning Feed Minute (0-59)", String(config.feedTime1Minute).c_str(), 3);
  WiFiManagerParameter custom_feed2_hour("feed2h", "Evening Feed Hour (0-23)", String(config.feedTime2Hour).c_str(), 3);
  WiFiManagerParameter custom_feed2_min("feed2m", "Evening Feed Minute (0-59)", String(config.feedTime2Minute).c_str(), 3);
  WiFiManagerParameter custom_target_temp("temp", "Target Temperature (°C)", String(config.targetTemp).c_str(), 5);
  WiFiManagerParameter custom_timezone("tz", "Timezone Offset (hours)", String(config.timezoneOffset).c_str(), 4);

  wifiManager.addParameter(&custom_feed1_hour);
  wifiManager.addParameter(&custom_feed1_min);
  wifiManager.addParameter(&custom_feed2_hour);
  wifiManager.addParameter(&custom_feed2_min);
  wifiManager.addParameter(&custom_target_temp);
  wifiManager.addParameter(&custom_timezone);

  // Try to connect, or start AP
  if (!wifiManager.autoConnect("CatFeeder-Setup")) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }

  // Save custom parameters if entered
  if (strlen(custom_feed1_hour.getValue()) > 0) {
    config.feedTime1Hour = String(custom_feed1_hour.getValue()).toInt();
    config.feedTime1Minute = String(custom_feed1_min.getValue()).toInt();
    config.feedTime2Hour = String(custom_feed2_hour.getValue()).toInt();
    config.feedTime2Minute = String(custom_feed2_min.getValue()).toInt();
    config.targetTemp = String(custom_target_temp.getValue()).toFloat();
    config.timezoneOffset = String(custom_timezone.getValue()).toInt();
    saveConfig();
  }

  Serial.println("WiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ============================================================================
// NTP SETUP
// ============================================================================
void setupNTP() {
  Serial.println("Synchronizing time with NTP...");
  configTime(config.timezoneOffset * 3600, 0, config.ntpServer);

  // Wait for time to be set
  int retries = 0;
  while (time(nullptr) < 100000 && retries < 10) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();

  time_t now = time(nullptr);
  Serial.println("Time synchronized: " + String(ctime(&now)));
}

// ============================================================================
// CONFIGURATION MANAGEMENT
// ============================================================================
void loadConfig() {
  preferences.begin("catfeeder", false);

  config.feedTime1Hour = preferences.getInt("feed1h", 7);
  config.feedTime1Minute = preferences.getInt("feed1m", 0);
  config.feedTime2Hour = preferences.getInt("feed2h", 18);
  config.feedTime2Minute = preferences.getInt("feed2m", 0);
  config.targetTemp = preferences.getFloat("targetTemp", 15.0);
  config.timezoneOffset = preferences.getInt("timezone", 0);
  config.calibrationOffset = preferences.getInt("calibration", 0);
  config.enableCooling = preferences.getBool("cooling", true);

  String ntpServer = preferences.getString("ntpServer", "pool.ntp.org");
  strcpy(config.ntpServer, ntpServer.c_str());

  preferences.end();

  Serial.println("Configuration loaded");
}

void saveConfig() {
  preferences.begin("catfeeder", false);

  preferences.putInt("feed1h", config.feedTime1Hour);
  preferences.putInt("feed1m", config.feedTime1Minute);
  preferences.putInt("feed2h", config.feedTime2Hour);
  preferences.putInt("feed2m", config.feedTime2Minute);
  preferences.putFloat("targetTemp", config.targetTemp);
  preferences.putInt("timezone", config.timezoneOffset);
  preferences.putInt("calibration", config.calibrationOffset);
  preferences.putBool("cooling", config.enableCooling);
  preferences.putString("ntpServer", config.ntpServer);

  preferences.end();

  Serial.println("Configuration saved");
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================
void startMotor() {
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  ledcWrite(MOTOR_PWM_CHANNEL, MOTOR_SPEED);
  motorRunning = true;
  motorStartTime = millis();
  Serial.println("Motor started");
}

void stopMotor() {
  ledcWrite(MOTOR_PWM_CHANNEL, 0);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  motorRunning = false;
  Serial.println("Motor stopped");
}

// ============================================================================
// CAROUSEL ROTATION
// ============================================================================
void rotateCarousel() {
  if (feedingInProgress) {
    Serial.println("Feeding already in progress");
    return;
  }

  feedingInProgress = true;
  waitingForEdge = true;

  Serial.println("Starting carousel rotation...");
  Serial.print("Current slot: ");
  Serial.println(currentSlot);

  // Start motor
  startMotor();

  // Read initial photocell state
  bool initialState = digitalRead(PHOTOCELL_PIN) == LOW;
  photocellTriggered = initialState;

  Serial.print("Initial photocell state: ");
  Serial.println(initialState ? "TRIGGERED" : "CLEAR");

  // Wait for photocell state change in main loop
  unsigned long rotationStart = millis();
  bool edgeDetected = false;
  bool slotDetected = false;

  // First, wait to clear the current position if sensor is already triggered
  if (initialState) {
    Serial.println("Waiting to clear current position...");
    while (digitalRead(PHOTOCELL_PIN) == LOW && (millis() - rotationStart) < MOTOR_TIMEOUT) {
      delay(10);
    }
    Serial.println("Current position cleared");
  }

  // Now wait for the next slot
  Serial.println("Waiting for next slot...");
  while (!slotDetected && (millis() - rotationStart) < MOTOR_TIMEOUT) {
    bool currentState = digitalRead(PHOTOCELL_PIN) == LOW;

    // Detect rising edge (transition to triggered state)
    if (currentState && !photocellTriggered) {
      Serial.println("Next slot detected!");
      slotDetected = true;

      // Apply calibration offset
      if (config.calibrationOffset > 0) {
        delay(config.calibrationOffset);
      }

      break;
    }

    photocellTriggered = currentState;
    delay(10);
  }

  // Stop motor
  stopMotor();

  if (slotDetected) {
    currentSlot = (currentSlot + 1) % SLOTS_COUNT;
    Serial.print("Rotation complete. New slot: ");
    Serial.println(currentSlot);
  } else {
    Serial.println("Rotation failed - timeout or sensor error");
  }

  feedingInProgress = false;
  waitingForEdge = false;
}

// ============================================================================
// FEEDING SCHEDULE
// ============================================================================
void checkFeedingSchedule() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);

  int currentHour = timeinfo->tm_hour;
  int currentMinute = timeinfo->tm_min;

  // Check first feeding time
  if (currentHour == config.feedTime1Hour && currentMinute == config.feedTime1Minute) {
    if (!lastFeedTime1Done) {
      Serial.println("Time for first feeding!");
      rotateCarousel();
      lastFeedTime1Done = true;
    }
  } else {
    lastFeedTime1Done = false;
  }

  // Check second feeding time
  if (currentHour == config.feedTime2Hour && currentMinute == config.feedTime2Minute) {
    if (!lastFeedTime2Done) {
      Serial.println("Time for second feeding!");
      rotateCarousel();
      lastFeedTime2Done = true;
    }
  } else {
    lastFeedTime2Done = false;
  }
}

// ============================================================================
// TEMPERATURE CONTROL
// ============================================================================
void readTemperature() {
  tempSensor.requestTemperatures();
  float temp = tempSensor.getTempCByIndex(0);

  if (temp != DEVICE_DISCONNECTED_C && temp > -50 && temp < 100) {
    currentTemp = temp;
    Serial.print("Temperature: ");
    Serial.print(currentTemp);
    Serial.println(" °C");
  } else {
    Serial.println("Error reading temperature sensor");
  }
}

void updateTemperatureControl() {
  if (!config.enableCooling) {
    ledcWrite(TEC_PWM_CHANNEL, 0);
    ledcWrite(FAN_PWM_CHANNEL, 0);
    return;
  }

  // Calculate TEC power using PID control
  int tecPower = calculateTECPower(currentTemp);
  ledcWrite(TEC_PWM_CHANNEL, tecPower);

  // Calculate fan speed proportional to temperature
  int fanSpeed = calculateFanSpeed(currentTemp);
  ledcWrite(FAN_PWM_CHANNEL, fanSpeed);

  Serial.print("TEC: ");
  Serial.print(tecPower);
  Serial.print(" | Fan: ");
  Serial.println(fanSpeed);
}

int calculateTECPower(float temp) {
  float error = temp - config.targetTemp;

  // Only cool if temperature is above target
  if (error <= 0) {
    pidIntegral = 0;
    pidLastError = 0;
    return 0;
  }

  // PID calculation
  pidIntegral += error * (TEMP_CHECK_INTERVAL / 1000.0);
  float derivative = (error - pidLastError) / (TEMP_CHECK_INTERVAL / 1000.0);
  pidLastError = error;

  float output = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);

  // Constrain output to 0-255
  int power = constrain((int)output, 0, 255);

  return power;
}

int calculateFanSpeed(float temp) {
  float tempDiff = temp - config.targetTemp;

  if (tempDiff <= 0) {
    return 0;
  }

  // Proportional fan speed: 0°C diff = min speed, 10°C+ diff = max speed
  int speed = map(constrain(tempDiff * 10, 0, 100), 0, 100, FAN_MIN_SPEED, FAN_MAX_SPEED);

  return speed;
}

// ============================================================================
// WEB SERVER SETUP
// ============================================================================
void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/saveconfig", HTTP_POST, handleSaveConfig);
  server.on("/feed", handleManualFeed);
  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Web server started");
}

// ============================================================================
// WEB SERVER HANDLERS
// ============================================================================
void handleRoot() {
  server.send(200, "text/html", getHTMLPage());
}

void handleConfig() {
  String html = getHTMLPage();
  server.send(200, "text/html", html);
}

void handleSaveConfig() {
  if (server.hasArg("feed1h")) config.feedTime1Hour = server.arg("feed1h").toInt();
  if (server.hasArg("feed1m")) config.feedTime1Minute = server.arg("feed1m").toInt();
  if (server.hasArg("feed2h")) config.feedTime2Hour = server.arg("feed2h").toInt();
  if (server.hasArg("feed2m")) config.feedTime2Minute = server.arg("feed2m").toInt();
  if (server.hasArg("temp")) config.targetTemp = server.arg("temp").toFloat();
  if (server.hasArg("tz")) config.timezoneOffset = server.arg("tz").toInt();
  if (server.hasArg("cal")) config.calibrationOffset = server.arg("cal").toInt();
  if (server.hasArg("cooling")) config.enableCooling = server.arg("cooling") == "on";

  saveConfig();

  server.send(200, "text/html", "<html><body><h2>Configuration Saved!</h2><p>Redirecting...</p><script>setTimeout(function(){window.location='/';}, 2000);</script></body></html>");
}

void handleManualFeed() {
  if (!feedingInProgress) {
    rotateCarousel();
    server.send(200, "text/plain", "Feeding initiated");
  } else {
    server.send(200, "text/plain", "Feeding already in progress");
  }
}

void handleStatus() {
  server.send(200, "application/json", getStatusJSON());
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

// ============================================================================
// HTML PAGE GENERATION
// ============================================================================
String getHTMLPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Cat Feeder Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      max-width: 800px;
      margin: 0 auto;
      padding: 20px;
      background-color: #f0f0f0;
    }
    .container {
      background-color: white;
      border-radius: 10px;
      padding: 20px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.1);
      margin-bottom: 20px;
    }
    h1 {
      color: #333;
      text-align: center;
      margin-top: 0;
    }
    h2 {
      color: #666;
      border-bottom: 2px solid #4CAF50;
      padding-bottom: 10px;
    }
    .status-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 15px;
      margin: 20px 0;
    }
    .status-item {
      background-color: #f9f9f9;
      padding: 15px;
      border-radius: 5px;
      border-left: 4px solid #4CAF50;
    }
    .status-label {
      font-weight: bold;
      color: #666;
      font-size: 0.9em;
    }
    .status-value {
      font-size: 1.5em;
      color: #333;
      margin-top: 5px;
    }
    .form-group {
      margin-bottom: 15px;
    }
    label {
      display: block;
      margin-bottom: 5px;
      color: #666;
      font-weight: bold;
    }
    input[type="number"], input[type="text"] {
      width: 100%;
      padding: 8px;
      border: 1px solid #ddd;
      border-radius: 4px;
      box-sizing: border-box;
    }
    .time-input {
      display: flex;
      gap: 10px;
    }
    .time-input input {
      flex: 1;
    }
    button {
      background-color: #4CAF50;
      color: white;
      padding: 12px 24px;
      border: none;
      border-radius: 4px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      margin-top: 10px;
    }
    button:hover {
      background-color: #45a049;
    }
    .button-secondary {
      background-color: #008CBA;
    }
    .button-secondary:hover {
      background-color: #007399;
    }
    .checkbox-group {
      display: flex;
      align-items: center;
      gap: 10px;
    }
    .checkbox-group input[type="checkbox"] {
      width: 20px;
      height: 20px;
    }
  </style>
  <script>
    function updateStatus() {
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          document.getElementById('temp').textContent = data.temperature + ' °C';
          document.getElementById('slot').textContent = data.currentSlot;
          document.getElementById('nextFeed').textContent = data.nextFeedTime;
          document.getElementById('tecPower').textContent = data.tecPower;
          document.getElementById('fanSpeed').textContent = data.fanSpeed;
        })
        .catch(error => console.error('Error:', error));
    }

    function manualFeed() {
      if (confirm('Start manual feeding cycle?')) {
        fetch('/feed')
          .then(response => response.text())
          .then(data => {
            alert(data);
            updateStatus();
          })
          .catch(error => {
            alert('Error: ' + error);
          });
      }
    }

    setInterval(updateStatus, 5000);
    window.onload = updateStatus;
  </script>
</head>
<body>
  <div class="container">
    <h1>🐱 Smart Cat Feeder</h1>
  </div>

  <div class="container">
    <h2>Status</h2>
    <div class="status-grid">
      <div class="status-item">
        <div class="status-label">Temperature</div>
        <div class="status-value" id="temp">--</div>
      </div>
      <div class="status-item">
        <div class="status-label">Current Slot</div>
        <div class="status-value" id="slot">--</div>
      </div>
      <div class="status-item">
        <div class="status-label">Next Feed Time</div>
        <div class="status-value" id="nextFeed">--</div>
      </div>
      <div class="status-item">
        <div class="status-label">TEC Power</div>
        <div class="status-value" id="tecPower">--</div>
      </div>
      <div class="status-item">
        <div class="status-label">Fan Speed</div>
        <div class="status-value" id="fanSpeed">--</div>
      </div>
    </div>
    <button class="button-secondary" onclick="manualFeed()">Manual Feed Now</button>
  </div>

  <div class="container">
    <h2>Configuration</h2>
    <form action="/saveconfig" method="POST">
      <div class="form-group">
        <label>Morning Feed Time (HH:MM)</label>
        <div class="time-input">
          <input type="number" name="feed1h" min="0" max="23" value=")rawliteral" +
          String(config.feedTime1Hour) + R"rawliteral(" placeholder="Hour">
          <input type="number" name="feed1m" min="0" max="59" value=")rawliteral" +
          String(config.feedTime1Minute) + R"rawliteral(" placeholder="Minute">
        </div>
      </div>

      <div class="form-group">
        <label>Evening Feed Time (HH:MM)</label>
        <div class="time-input">
          <input type="number" name="feed2h" min="0" max="23" value=")rawliteral" +
          String(config.feedTime2Hour) + R"rawliteral(" placeholder="Hour">
          <input type="number" name="feed2m" min="0" max="59" value=")rawliteral" +
          String(config.feedTime2Minute) + R"rawliteral(" placeholder="Minute">
        </div>
      </div>

      <div class="form-group">
        <label>Target Temperature (°C)</label>
        <input type="number" name="temp" step="0.1" value=")rawliteral" +
        String(config.targetTemp) + R"rawliteral(">
      </div>

      <div class="form-group">
        <label>Timezone Offset (hours from UTC)</label>
        <input type="number" name="tz" min="-12" max="14" value=")rawliteral" +
        String(config.timezoneOffset) + R"rawliteral(">
      </div>

      <div class="form-group">
        <label>Calibration Offset (milliseconds)</label>
        <input type="number" name="cal" min="-500" max="500" value=")rawliteral" +
        String(config.calibrationOffset) + R"rawliteral(">
      </div>

      <div class="form-group">
        <label class="checkbox-group">
          <input type="checkbox" name="cooling" )rawliteral" +
          String(config.enableCooling ? "checked" : "") + R"rawliteral(>
          <span>Enable TEC Cooling</span>
        </label>
      </div>

      <button type="submit">Save Configuration</button>
    </form>
  </div>

  <div class="container" style="text-align: center; color: #999; font-size: 0.9em;">
    <p>Smart Cat Feeder v1.0 | ESP32</p>
  </div>
</body>
</html>
)rawliteral";

  return html;
}

// ============================================================================
// STATUS JSON GENERATION
// ============================================================================
String getStatusJSON() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);

  // Calculate next feed time
  String nextFeed = "";
  int currentMinutes = timeinfo->tm_hour * 60 + timeinfo->tm_min;
  int feed1Minutes = config.feedTime1Hour * 60 + config.feedTime1Minute;
  int feed2Minutes = config.feedTime2Hour * 60 + config.feedTime2Minute;

  if (currentMinutes < feed1Minutes) {
    nextFeed = String(config.feedTime1Hour) + ":" +
               (config.feedTime1Minute < 10 ? "0" : "") + String(config.feedTime1Minute);
  } else if (currentMinutes < feed2Minutes) {
    nextFeed = String(config.feedTime2Hour) + ":" +
               (config.feedTime2Minute < 10 ? "0" : "") + String(config.feedTime2Minute);
  } else {
    nextFeed = String(config.feedTime1Hour) + ":" +
               (config.feedTime1Minute < 10 ? "0" : "") + String(config.feedTime1Minute) + " (tomorrow)";
  }

  String json = "{";
  json += "\"temperature\":" + String(currentTemp, 1) + ",";
  json += "\"currentSlot\":" + String(currentSlot) + ",";
  json += "\"nextFeedTime\":\"" + nextFeed + "\",";
  json += "\"tecPower\":" + String(calculateTECPower(currentTemp)) + ",";
  json += "\"fanSpeed\":" + String(calculateFanSpeed(currentTemp)) + ",";
  json += "\"feedingInProgress\":" + String(feedingInProgress ? "true" : "false");
  json += "}";

  return json;
}
)rawliteral";

  return html;
}

String getStatusJSON() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);

  // Calculate next feed time
  String nextFeed = "";
  int currentMinutes = timeinfo->tm_hour * 60 + timeinfo->tm_min;
  int feed1Minutes = config.feedTime1Hour * 60 + config.feedTime1Minute;
  int feed2Minutes = config.feedTime2Hour * 60 + config.feedTime2Minute;

  if (currentMinutes < feed1Minutes) {
    nextFeed = String(config.feedTime1Hour) + ":" +
               (config.feedTime1Minute < 10 ? "0" : "") + String(config.feedTime1Minute);
  } else if (currentMinutes < feed2Minutes) {
    nextFeed = String(config.feedTime2Hour) + ":" +
               (config.feedTime2Minute < 10 ? "0" : "") + String(config.feedTime2Minute);
  } else {
    nextFeed = String(config.feedTime1Hour) + ":" +
               (config.feedTime1Minute < 10 ? "0" : "") + String(config.feedTime1Minute) + " (tomorrow)";
  }

  String json = "{";
  json += "\"temperature\":" + String(currentTemp, 1) + ",";
  json += "\"currentSlot\":" + String(currentSlot) + ",";
  json += "\"nextFeedTime\":\"" + nextFeed + "\",";
  json += "\"tecPower\":" + String(calculateTECPower(currentTemp)) + ",";
  json += "\"fanSpeed\":" + String(calculateFanSpeed(currentTemp)) + ",";
  json += "\"feedingInProgress\":" + String(feedingInProgress ? "true" : "false");
  json += "}";

  return json;
}
