/*
 * Arduino Opta Water Pressure Controller
 * With Home Assistant MQTT Auto-Discovery & Fault Protection
 *
 * Features:
 * - Reads 0-10V pressure sensor on I1 (A0)
 * - Reads 0-10V / 0-20A current sensor on I2 (A1)
 * - Controls pump via Relay 1 (D0)
 * - Fault indicator light on Relay 2 (D1)
 * - Front panel status LEDs:
 *     LED 1: Low Pressure fault
 *     LED 2: Overcurrent fault
 *     LED 3: Undercurrent fault
 *     LED 4: System OK (no faults)
 * - Publishes pressure, amps, pump status, fault status to MQTT
 * - Subscribes to MQTT for pump on/off and reset commands
 * - Home Assistant auto-discovery
 * - Non-blocking network: local control works even if network is down
 *
 * Fault Protection:
 * - LOW PRESSURE:  Faults if pressure < 30 PSI while pump is running
 * - OVERCURRENT:   Faults if amps > 10A for more than 6 seconds
 * - UNDERCURRENT:  Faults if amps <= 3A for more than 6 seconds while running
 * - Reset via physical button on I3 (A2) or MQTT reset command
 * - JOG/BYPASS on I4 (A3): Hold to run pump, bypasses low pressure & undercurrent
 *   (Overcurrent still protected for safety)
 *
 * MQTT Topics:
 *   Publish:
 *     opta/pump/pressure     - Current pressure (PSI)
 *     opta/pump/amps         - Current draw in amps
 *     opta/pump/status       - "ON" or "OFF"
 *     opta/pump/fault        - "OK", "LOW_PRESSURE", "OVERCURRENT", or "UNDERCURRENT"
 *     opta/pump/availability - "online" or "offline"
 *   Subscribe:
 *     opta/pump/command      - "ON" or "OFF" to control pump
 *     opta/pump/reset        - "RESET" to clear fault
 */

/*
 * BOARD SETUP - IMPORTANT!
 * ========================
 * This code requires the "Arduino Mbed OS Opta Boards" core (stable).
 * The newer Zephyr core (0.52.0) has different Ethernet APIs.
 *
 * To install the correct core:
 * 1. Tools > Board > Boards Manager
 * 2. Search "Mbed OS Opta"
 * 3. Install "Arduino Mbed OS Opta Boards" (NOT the Zephyr version)
 * 4. Select: Tools > Board > Arduino Mbed OS Opta Boards > Arduino Opta
 *
 * Required Libraries (via Library Manager):
 * - ArduinoMqttClient
 */

#include <Ethernet.h>
#include <ArduinoMqttClient.h>

// ============== OPTA STATUS LED PINS ==============
// The 4 green status LEDs on the Opta front panel
// Using direct pin numbers that work across cores
const int LED_STATUS_1 = 7;    // Status LED 1 (near relay 1)
const int LED_STATUS_2 = 8;    // Status LED 2
const int LED_STATUS_3 = 9;    // Status LED 3
const int LED_STATUS_4 = 10;   // Status LED 4 (near relay 4)

// ============== CONFIGURATION ==============

// Network Settings
byte mac[] = { 0xA8, 0x61, 0x0A, 0x50, 0xA6, 0xB7 };
IPAddress ip(10, 58, 82, 99);
IPAddress gateway(10, 58, 82, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT Settings
const char* mqttBroker = "10.58.82.15";
const int mqttPort = 1883;
const char* mqttUser = "mqtt-opta-pumpcontroller";
const char* mqttPass = "8u074mE6c7DboefkasSJff8IpdjptEJ4w";

// Device Info (for Home Assistant)
const char* deviceName = "Water Pump Controller";
const char* deviceId = "opta_pump";
const char* manufacturer = "Arduino";
const char* model = "Opta";

// MQTT Topics
const char* topicPressure     = "opta/pump/pressure";
const char* topicAmps         = "opta/pump/amps";
const char* topicStatus       = "opta/pump/status";
const char* topicFault        = "opta/pump/fault";
const char* topicCommand      = "opta/pump/command";
const char* topicReset        = "opta/pump/reset";
const char* topicAvailability = "opta/pump/availability";

// Sensor Calibration
const float PRESSURE_MAX = 87.0;
const float PRESSURE_MIN = 0.0;
const float AMP_MAX = 20.0;
const float AMP_MIN = 0.0;

// Fault Thresholds
const float LOW_PRESSURE_THRESHOLD = 30.0;    // PSI - fault if below this
const float OVERCURRENT_THRESHOLD = 10.0;     // Amps - fault if above this
const float UNDERCURRENT_THRESHOLD = 3.0;     // Amps - fault if below this when running
const unsigned long UNDERCURRENT_DELAY = 6000; // ms - must be below for this duration
const unsigned long OVERCURRENT_DELAY = 6000; // ms - must exceed for this duration

// Timing
const unsigned long PUBLISH_INTERVAL = 2000;
const unsigned long RECONNECT_INTERVAL = 5000;
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long ETH_CHECK_INTERVAL = 1000;
const int MQTT_CONNECT_TIMEOUT = 2000;  // ms - non-blocking timeout

// ============== PIN DEFINITIONS ==============

// Analog Inputs
#define PRESSURE_PIN A0      // I1 - Pressure sensor 0-10V
#define AMP_PIN      A1      // I2 - Current sensor 0-10V

// Digital Inputs
#define RESET_BUTTON_PIN A2  // I3 - Reset button (active LOW with internal pull-up)
#define JOG_BUTTON_PIN   A3  // I4 - Jog/Bypass button (hold to run pump, bypasses low pressure)

// Relay Outputs
#define PUMP_RELAY  D0       // Relay 1 - Pump control
#define FAULT_LIGHT D1       // Relay 2 - Fault indicator light

// Status LEDs (front panel) - mapped to Opta status LEDs
#define LED_LOW_PRESSURE  LED_STATUS_1   // LED 1 - Low pressure fault
#define LED_OVERCURRENT   LED_STATUS_2   // LED 2 - Overcurrent fault
#define LED_UNDERCURRENT  LED_STATUS_3   // LED 3 - Undercurrent fault
#define LED_SYSTEM_OK     LED_STATUS_4   // LED 4 - System OK (no fault)

// ============== FAULT STATES ==============

enum FaultState {
  FAULT_NONE,
  FAULT_LOW_PRESSURE,
  FAULT_OVERCURRENT,
  FAULT_UNDERCURRENT
};

// ============== GLOBAL VARIABLES ==============

EthernetClient ethClient;
MqttClient mqttClient(ethClient);

bool pumpState = false;
bool pumpRequested = false;  // User's desired pump state
float currentPressure = 0.0;
float currentAmps = 0.0;

FaultState faultState = FAULT_NONE;
unsigned long overcurrentStartTime = 0;
bool overcurrentTiming = false;
unsigned long undercurrentStartTime = 0;
bool undercurrentTiming = false;

unsigned long lastPublish = 0;
unsigned long lastReconnect = 0;
unsigned long lastButtonRead = 0;
unsigned long lastEthCheck = 0;
bool lastButtonState = HIGH;
bool ethConnected = false;
bool mqttConfigured = false;
bool jogMode = false;  // True when jog button is held - bypasses low pressure/undercurrent

// ============== SETUP ==============

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("=================================");
  Serial.println("Opta Water Pressure Controller");
  Serial.println("With Fault Protection");
  Serial.println("=================================");

  // Initialize relay outputs
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(FAULT_LIGHT, OUTPUT);
  digitalWrite(PUMP_RELAY, LOW);
  digitalWrite(FAULT_LIGHT, LOW);
  pumpState = false;
  pumpRequested = false;

  // Initialize status LEDs
  pinMode(LED_LOW_PRESSURE, OUTPUT);
  pinMode(LED_OVERCURRENT, OUTPUT);
  pinMode(LED_UNDERCURRENT, OUTPUT);
  pinMode(LED_SYSTEM_OK, OUTPUT);
  updateStatusLEDs();  // Set initial LED state

  // Initialize button inputs (active LOW with internal pull-up)
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(JOG_BUTTON_PIN, INPUT_PULLUP);

  // Configure analog resolution
  analogReadResolution(12);

  // Initialize Ethernet & MQTT
  initEthernet();
  initMQTT();

  Serial.println("System ready (non-blocking network).");
  Serial.print("Low pressure threshold: ");
  Serial.print(LOW_PRESSURE_THRESHOLD);
  Serial.println(" PSI");
  Serial.print("Overcurrent threshold: ");
  Serial.print(OVERCURRENT_THRESHOLD);
  Serial.print(" A for ");
  Serial.print(OVERCURRENT_DELAY / 1000);
  Serial.println(" seconds");
  Serial.print("Undercurrent threshold: ");
  Serial.print(UNDERCURRENT_THRESHOLD);
  Serial.print(" A for ");
  Serial.print(UNDERCURRENT_DELAY / 1000);
  Serial.println(" seconds");
  Serial.println("Local control active - network connects in background.");
}

// ============== MAIN LOOP ==============

void loop() {
  // === CRITICAL: Local control always runs first (non-blocking) ===

  // Read sensors
  readSensors();

  // Check for reset button press
  checkResetButton();

  // Check for jog button (bypass low pressure to prime pump)
  checkJogButton();

  // Check for fault conditions (respects jog mode)
  checkFaults();

  // Control pump based on state and faults
  updatePumpOutput();

  // === Network tasks (non-blocking) ===

  // Check Ethernet connection status
  checkEthernet();

  // Maintain MQTT connection (only if Ethernet is up)
  if (ethConnected && mqttConfigured) {
    if (!mqttClient.connected()) {
      unsigned long now = millis();
      if (now - lastReconnect >= RECONNECT_INTERVAL) {
        lastReconnect = now;
        reconnectMQTT();
      }
    } else {
      mqttClient.poll();
    }
  }

  // Publish data at interval (only if connected)
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publishData();
  }
}

// ============== ETHERNET (NON-BLOCKING) ==============

void initEthernet() {
  Serial.println("Initializing Ethernet (non-blocking)...");
  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  // No delay - connection state checked in loop
}

void checkEthernet() {
  unsigned long now = millis();
  if (now - lastEthCheck < ETH_CHECK_INTERVAL) return;
  lastEthCheck = now;

  // Check if we have a valid IP (link is up)
  bool linkUp = (Ethernet.linkStatus() == LinkON);

  if (linkUp && !ethConnected) {
    ethConnected = true;
    Serial.print("Ethernet connected. IP: ");
    Serial.println(Ethernet.localIP());
  } else if (!linkUp && ethConnected) {
    ethConnected = false;
    Serial.println("Ethernet disconnected!");
  }

  // Maintain Ethernet stack
  Ethernet.maintain();
}

// ============== MQTT (NON-BLOCKING) ==============

void initMQTT() {
  if (strlen(mqttUser) > 0) {
    mqttClient.setUsernamePassword(mqttUser, mqttPass);
  }

  mqttClient.beginWill(topicAvailability, true, 1);
  mqttClient.print("offline");
  mqttClient.endWill();

  // Set connection timeout to prevent blocking
  mqttClient.setConnectionTimeout(MQTT_CONNECT_TIMEOUT);

  mqttClient.onMessage(onMqttMessage);
  mqttConfigured = true;
  // Don't connect here - let loop() handle it non-blocking
}

void reconnectMQTT() {
  // Skip if Ethernet not connected
  if (!ethConnected) return;

  Serial.print("MQTT connecting... ");

  // This will timeout after MQTT_CONNECT_TIMEOUT ms
  if (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print("Failed: ");
    Serial.println(mqttClient.connectError());
    return;
  }

  Serial.println("Connected!");

  // Publish availability
  mqttClient.beginMessage(topicAvailability, true, 1);
  mqttClient.print("online");
  mqttClient.endMessage();

  // Subscribe to topics
  mqttClient.subscribe(topicCommand);
  mqttClient.subscribe(topicReset);
  Serial.println("Subscribed to command and reset topics");

  // Send Home Assistant discovery configs
  sendHADiscovery();

  // Publish initial status
  publishAllStatus();
}

void onMqttMessage(int messageSize) {
  String topic = mqttClient.messageTopic();

  String message = "";
  while (mqttClient.available()) {
    message += (char)mqttClient.read();
  }
  message.toUpperCase();
  message.trim();

  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  // Handle pump commands
  if (topic == topicCommand) {
    if (faultState != FAULT_NONE) {
      Serial.println("Command ignored - system faulted. Send RESET first.");
      return;
    }
    if (message == "ON" || message == "1" || message == "START") {
      pumpRequested = true;
    } else if (message == "OFF" || message == "0" || message == "STOP") {
      pumpRequested = false;
    } else if (message == "TOGGLE") {
      pumpRequested = !pumpRequested;
    }
  }

  // Handle reset command
  if (topic == topicReset) {
    if (message == "RESET" || message == "1" || message == "CLEAR") {
      resetFault();
    }
  }
}

// ============== HOME ASSISTANT DISCOVERY ==============

void sendHADiscovery() {
  Serial.println("Sending Home Assistant discovery configs...");

  String deviceJson = String("\"dev\":{") +
    "\"ids\":[\"" + deviceId + "\"]," +
    "\"name\":\"" + deviceName + "\"," +
    "\"mf\":\"" + manufacturer + "\"," +
    "\"mdl\":\"" + model + "\"" +
    "}";

  String availabilityJson = String("\"avty_t\":\"") + topicAvailability + "\","
    "\"pl_avail\":\"online\","
    "\"pl_not_avail\":\"offline\"";

  // --- Pressure Sensor ---
  String pressureConfig = String("{") +
    "\"name\":\"Water Pressure\"," +
    "\"uniq_id\":\"" + deviceId + "_pressure\"," +
    "\"stat_t\":\"" + topicPressure + "\"," +
    "\"unit_of_meas\":\"PSI\"," +
    "\"dev_cla\":\"pressure\"," +
    "\"stat_cla\":\"measurement\"," +
    "\"icon\":\"mdi:gauge\"," +
    availabilityJson + "," +
    deviceJson +
    "}";

  mqttClient.beginMessage("homeassistant/sensor/opta_pump_pressure/config", true, 1);
  mqttClient.print(pressureConfig);
  mqttClient.endMessage();

  // --- Amps Sensor ---
  String ampsConfig = String("{") +
    "\"name\":\"Pump Current\"," +
    "\"uniq_id\":\"" + deviceId + "_amps\"," +
    "\"stat_t\":\"" + topicAmps + "\"," +
    "\"unit_of_meas\":\"A\"," +
    "\"dev_cla\":\"current\"," +
    "\"stat_cla\":\"measurement\"," +
    "\"icon\":\"mdi:current-ac\"," +
    availabilityJson + "," +
    deviceJson +
    "}";

  mqttClient.beginMessage("homeassistant/sensor/opta_pump_amps/config", true, 1);
  mqttClient.print(ampsConfig);
  mqttClient.endMessage();

  // --- Pump Switch ---
  String switchConfig = String("{") +
    "\"name\":\"Water Pump\"," +
    "\"uniq_id\":\"" + deviceId + "_switch\"," +
    "\"stat_t\":\"" + topicStatus + "\"," +
    "\"cmd_t\":\"" + topicCommand + "\"," +
    "\"pl_on\":\"ON\"," +
    "\"pl_off\":\"OFF\"," +
    "\"stat_on\":\"ON\"," +
    "\"stat_off\":\"OFF\"," +
    "\"icon\":\"mdi:water-pump\"," +
    availabilityJson + "," +
    deviceJson +
    "}";

  mqttClient.beginMessage("homeassistant/switch/opta_pump/config", true, 1);
  mqttClient.print(switchConfig);
  mqttClient.endMessage();

  // --- Fault Status Sensor ---
  String faultConfig = String("{") +
    "\"name\":\"Pump Fault\"," +
    "\"uniq_id\":\"" + deviceId + "_fault\"," +
    "\"stat_t\":\"" + topicFault + "\"," +
    "\"icon\":\"mdi:alert-circle\"," +
    availabilityJson + "," +
    deviceJson +
    "}";

  mqttClient.beginMessage("homeassistant/sensor/opta_pump_fault/config", true, 1);
  mqttClient.print(faultConfig);
  mqttClient.endMessage();

  // --- Reset Button (as a button entity in HA) ---
  String resetConfig = String("{") +
    "\"name\":\"Pump Reset\"," +
    "\"uniq_id\":\"" + deviceId + "_reset\"," +
    "\"cmd_t\":\"" + topicReset + "\"," +
    "\"pl_prs\":\"RESET\"," +
    "\"icon\":\"mdi:restart\"," +
    availabilityJson + "," +
    deviceJson +
    "}";

  mqttClient.beginMessage("homeassistant/button/opta_pump_reset/config", true, 1);
  mqttClient.print(resetConfig);
  mqttClient.endMessage();

  Serial.println("Home Assistant discovery complete!");
}

// ============== SENSOR READING ==============

void readSensors() {
  int rawPressure = analogRead(PRESSURE_PIN);
  int rawAmps = analogRead(AMP_PIN);

  float voltagePressure = (rawPressure / 4095.0) * 10.0;
  float voltageAmps = (rawAmps / 4095.0) * 10.0;

  currentPressure = mapFloat(voltagePressure, 0.0, 10.0, PRESSURE_MIN, PRESSURE_MAX);
  currentAmps = mapFloat(voltageAmps, 0.0, 10.0, AMP_MIN, AMP_MAX);

  currentPressure = constrain(currentPressure, PRESSURE_MIN, PRESSURE_MAX);
  currentAmps = constrain(currentAmps, AMP_MIN, AMP_MAX);
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// ============== RESET BUTTON ==============

void checkResetButton() {
  unsigned long now = millis();
  if (now - lastButtonRead < DEBOUNCE_DELAY) return;
  lastButtonRead = now;

  // Read button (active LOW with pull-up)
  bool buttonState = digitalRead(RESET_BUTTON_PIN);

  // Detect falling edge (button press)
  if (lastButtonState == HIGH && buttonState == LOW) {
    Serial.println("Reset button pressed");
    resetFault();
  }

  lastButtonState = buttonState;
}

// ============== JOG BUTTON (BYPASS LOW PRESSURE) ==============

void checkJogButton() {
  // Jog button is held = run pump bypassing low pressure & undercurrent faults
  // Still protects against overcurrent for safety
  bool jogPressed = (digitalRead(JOG_BUTTON_PIN) == LOW);

  if (jogPressed && !jogMode) {
    jogMode = true;
    pumpRequested = true;  // Start pump while jogging
    // Clear low pressure or undercurrent fault if present
    if (faultState == FAULT_LOW_PRESSURE || faultState == FAULT_UNDERCURRENT) {
      faultState = FAULT_NONE;
      digitalWrite(FAULT_LIGHT, LOW);
      updateStatusLEDs();
    }
    Serial.println("JOG MODE: Pump running (low pressure bypass)");
  } else if (!jogPressed && jogMode) {
    jogMode = false;
    pumpRequested = false;  // Stop pump when jog released
    Serial.println("JOG MODE: Released - pump stopped");
    publishPumpStatus();
  }
}

// ============== FAULT HANDLING ==============

void checkFaults() {
  // Skip if already faulted
  if (faultState != FAULT_NONE) return;

  // LOW PRESSURE: Only check when pump is running AND not in jog mode
  if (!jogMode && pumpState && currentPressure < LOW_PRESSURE_THRESHOLD) {
    triggerFault(FAULT_LOW_PRESSURE);
    return;
  }

  // OVERCURRENT: Must exceed threshold for OVERCURRENT_DELAY duration
  if (currentAmps > OVERCURRENT_THRESHOLD) {
    if (!overcurrentTiming) {
      overcurrentTiming = true;
      overcurrentStartTime = millis();
      Serial.println("Overcurrent detected, starting 6s timer...");
    } else {
      if (millis() - overcurrentStartTime >= OVERCURRENT_DELAY) {
        triggerFault(FAULT_OVERCURRENT);
        return;
      }
    }
  } else {
    if (overcurrentTiming) {
      Serial.println("Current normalized, overcurrent timer reset");
      overcurrentTiming = false;
    }
  }

  // UNDERCURRENT: Only check when pump is running AND not in jog mode
  if (!jogMode && pumpState && currentAmps <= UNDERCURRENT_THRESHOLD) {
    if (!undercurrentTiming) {
      undercurrentTiming = true;
      undercurrentStartTime = millis();
      Serial.println("Undercurrent detected, starting 6s timer...");
    } else {
      if (millis() - undercurrentStartTime >= UNDERCURRENT_DELAY) {
        triggerFault(FAULT_UNDERCURRENT);
      }
    }
  } else {
    if (undercurrentTiming) {
      Serial.println("Current normalized, undercurrent timer reset");
      undercurrentTiming = false;
    }
  }
}

void triggerFault(FaultState fault) {
  faultState = fault;
  pumpState = false;
  pumpRequested = false;
  digitalWrite(PUMP_RELAY, LOW);
  digitalWrite(FAULT_LIGHT, HIGH);  // Turn on fault indicator
  updateStatusLEDs();               // Update front panel LEDs

  const char* faultName = getFaultString(fault);
  Serial.print("!!! FAULT: ");
  Serial.println(faultName);

  // Immediately publish fault status
  publishFaultStatus();
  publishPumpStatus();
}

void resetFault() {
  if (faultState == FAULT_NONE) {
    Serial.println("No fault to reset");
    return;
  }

  Serial.println("Fault cleared");
  faultState = FAULT_NONE;
  overcurrentTiming = false;
  undercurrentTiming = false;
  pumpRequested = false;  // Require explicit ON command after reset
  digitalWrite(FAULT_LIGHT, LOW);  // Turn off fault indicator
  updateStatusLEDs();              // Update front panel LEDs

  publishFaultStatus();
  publishPumpStatus();
}

const char* getFaultString(FaultState fault) {
  switch (fault) {
    case FAULT_LOW_PRESSURE: return "LOW_PRESSURE";
    case FAULT_OVERCURRENT:  return "OVERCURRENT";
    case FAULT_UNDERCURRENT: return "UNDERCURRENT";
    default:                 return "OK";
  }
}

void updateStatusLEDs() {
  // Turn off all fault LEDs first
  digitalWrite(LED_LOW_PRESSURE, LOW);
  digitalWrite(LED_OVERCURRENT, LOW);
  digitalWrite(LED_UNDERCURRENT, LOW);
  digitalWrite(LED_SYSTEM_OK, LOW);

  // Light up the appropriate LED based on fault state
  switch (faultState) {
    case FAULT_LOW_PRESSURE:
      digitalWrite(LED_LOW_PRESSURE, HIGH);
      break;
    case FAULT_OVERCURRENT:
      digitalWrite(LED_OVERCURRENT, HIGH);
      break;
    case FAULT_UNDERCURRENT:
      digitalWrite(LED_UNDERCURRENT, HIGH);
      break;
    case FAULT_NONE:
    default:
      digitalWrite(LED_SYSTEM_OK, HIGH);  // System OK - no faults
      break;
  }
}

// ============== PUMP CONTROL ==============

void updatePumpOutput() {
  bool shouldRun = (faultState == FAULT_NONE) && pumpRequested;

  if (shouldRun != pumpState) {
    pumpState = shouldRun;
    digitalWrite(PUMP_RELAY, pumpState ? HIGH : LOW);

    Serial.print("Pump: ");
    Serial.println(pumpState ? "ON" : "OFF");

    publishPumpStatus();
  }
}

// ============== MQTT PUBLISHING ==============

void publishData() {
  // Always print status to serial (local monitoring)
  Serial.print("P: ");
  Serial.print(currentPressure, 1);
  Serial.print(" PSI | A: ");
  Serial.print(currentAmps, 2);
  Serial.print(" | Pump: ");
  Serial.print(pumpState ? "ON" : "OFF");
  if (jogMode) Serial.print(" [JOG]");
  Serial.print(" | Fault: ");
  Serial.print(getFaultString(faultState));
  Serial.print(" | Net: ");
  Serial.println(ethConnected ? (mqttClient.connected() ? "MQTT" : "ETH") : "DOWN");

  // Only publish to MQTT if connected
  if (!mqttClient.connected()) return;

  mqttClient.beginMessage(topicPressure);
  mqttClient.print(currentPressure, 2);
  mqttClient.endMessage();

  mqttClient.beginMessage(topicAmps);
  mqttClient.print(currentAmps, 2);
  mqttClient.endMessage();
}

void publishAllStatus() {
  publishPumpStatus();
  publishFaultStatus();
}

void publishPumpStatus() {
  if (!mqttClient.connected()) return;

  mqttClient.beginMessage(topicStatus, true, 1);
  mqttClient.print(pumpState ? "ON" : "OFF");
  mqttClient.endMessage();
}

void publishFaultStatus() {
  if (!mqttClient.connected()) return;

  mqttClient.beginMessage(topicFault, true, 1);
  mqttClient.print(getFaultString(faultState));
  mqttClient.endMessage();
}