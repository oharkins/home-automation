/*
 * Arduino Opta Water Pressure Controller
 * With Home Assistant MQTT Auto-Discovery & Fault Protection
 *
 * Features:
 * - Reads 0-10V pressure sensor on I1 (A0)
 * - Reads 0-10V / 0-20A current sensor on I2 (A1)
 * - Controls pump via Relay 1 (D0)
 * - Fault indicator light on Relay 2 (D1)
 * - User button (BTN_USER) for fault reset
 * - 3-position mode selector on I7 (A6) and I8 (A7)
 * - Front panel status LEDs:
 *     LED 1: Low Pressure fault
 *     LED 2: Overcurrent fault
 *     LED 3: Undercurrent fault
 *     LED 4: System OK (no faults)
 * - Publishes pressure, amps, pump status, fault status to MQTT
 * - Subscribes to MQTT for mode changes and reset commands
 * - Home Assistant auto-discovery
 * - Non-blocking network: local control works even if network is down
 *
 * Physical Control Panel Wiring:
 * ==============================
 * 
 * RESET BUTTON:
 * - Use the built-in USER button on the Opta (BTN_USER)
 * - Press to clear any fault condition
 * 
 * MODE SELECTOR (I7 / A6 and I8 / A7):
 * - REQUIRES EXTERNAL 10K PULL-UP RESISTORS (Opta analog inputs don't support internal pull-ups)
 * - Use a 3-position rotary switch with common ground
 * - Pin states determine operation mode:
 * 
 *   Position | Pin A (I7) | Pin B (I8) | Mode
 *   ---------|------------|------------|------
 *      1     |   HIGH     |   HIGH     | OFF
 *      2     |   LOW      |   HIGH     | ON (Manual/Priming)
 *      3     |   HIGH     |   LOW      | AUTO
 * 
 * Wiring Example for 3-Position Rotary Switch:
 *   - Add 10K resistor from I7 to +V (use +24V, +5V, or +3.3V available on Opta)
 *   - Add 10K resistor from I8 to +V
 *   - Connect common terminal of rotary switch to GND
 *   - Position 1: No connection (both pins HIGH via pull-ups) = OFF
 *   - Position 2: Connect to I7 (grounds Pin A) = ON
 *   - Position 3: Connect to I8 (grounds Pin B) = AUTO
 * 
 * Alternative: Use two SPST toggle switches:
 *   - Add 10K pull-up resistors as above
 *   - Switch A between I7 and GND
 *   - Switch B between I8 and GND
 *   - Both OFF (HIGH/HIGH) = OFF mode
 *   - Switch A ON (LOW/HIGH) = ON mode
 *   - Switch B ON (HIGH/LOW) = AUTO mode
 *
 * Operation Modes:
 * - OFF:  Pump is completely off, all fault checking disabled
 *         - Cycling to OFF mode automatically clears any active faults
 * - ON:   Pump runs continuously (manual/priming mode)
 *         - Ignores low pressure warnings (for priming)
 *         - Respects HIGH PRESSURE CUTOUT (stops at 60 PSI, auto-restarts when pressure drops)
 *         - Ignores undercurrent faults (for priming/variable loads)
 *         - Still respects overcurrent protection
 * - AUTO: Pump runs automatically with full fault protection
 *         - All fault checks active (low pressure, overcurrent, undercurrent)
 *         - Pump turns on automatically, stops only on fault
 *
 * Fault Protection:
 * - LOW PRESSURE:  Faults if pressure < 30 PSI while pump is running (AUTO mode only)
 * - HIGH PRESSURE: Cutout at 60 PSI in ON mode (not a fault, auto-restarts)
 * - OVERCURRENT:   Faults if amps > 10A for more than 6 seconds (ON and AUTO modes)
 * - UNDERCURRENT:  Faults if amps <= 3A for more than 6 seconds while running (AUTO mode only)
 * - Reset via USER button, cycling mode selector to OFF, or MQTT reset command
 *
 * MQTT Topics:
 *   Publish:
 *     opta/pump/pressure     - Current pressure (PSI)
 *     opta/pump/amps         - Current draw in amps
 *     opta/pump/status       - "ON" or "OFF"
 *     opta/pump/fault        - "OK", "LOW_PRESSURE", "OVERCURRENT", or "UNDERCURRENT"
 *     opta/pump/mode         - "OFF", "ON", or "AUTO"
 *     opta/pump/availability - "online" or "offline"
 *   Subscribe:
 *     opta/pump/reset        - "RESET" to clear fault
 *     opta/pump/mode/set     - "OFF", "ON", or "AUTO" to change operation mode
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
 * - Arduino_JSON
 */

#include <Ethernet.h>
#include <ArduinoMqttClient.h>
#include <Arduino_JSON.h>

// ============== OPTA STATUS LED PINS ==============
// The 4 green status LEDs on the Opta front panel
// These are predefined in the Opta core as LED_D0, LED_D1, LED_D2, LED_D3
// Mapping to our application:
// LED_D0 = Status LED 1 (Low Pressure fault indicator)
// LED_D1 = Status LED 2 (Overcurrent fault indicator)
// LED_D2 = Status LED 3 (Undercurrent fault indicator)
// LED_D3 = Status LED 4 (System OK indicator)

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
const char* topicReset        = "opta/pump/reset";
const char* topicAvailability = "opta/pump/availability";
const char* topicMode         = "opta/pump/mode";
const char* topicModeCommand  = "opta/pump/mode/set";

// Sensor Calibration
const float PRESSURE_MAX = 87.0;
const float PRESSURE_MIN = 0.0;
const float AMP_MAX = 20.0;
const float AMP_MIN = 0.0;

// Fault Thresholds
const float LOW_PRESSURE_THRESHOLD = 30.0;    // PSI - fault if below this
const float HIGH_PRESSURE_CUTOUT = 60.0;      // PSI - cutout if above this (ON mode)
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
#define MODE_SELECT_A    A6  // I7 - Mode selector bit 0 (REQUIRES EXTERNAL 10K PULL-UP to +3.3V or +5V)
#define MODE_SELECT_B    A7  // I8 - Mode selector bit 1 (REQUIRES EXTERNAL 10K PULL-UP to +3.3V or +5V)

// IMPORTANT: Opta analog inputs (A3-A7) do NOT support INPUT_PULLUP!
// You MUST add external 10K pull-up resistors:
//   - Connect 10K resistor from I7 to +V (use Opta's +24V or external +5V/+3.3V)
//   - Connect 10K resistor from I8 to +V
//   - Connect switches to pull pins to GND
//
// Mode selector truth table (active LOW with EXTERNAL pull-ups):
// A=HIGH, B=HIGH (11) = OFF mode
// A=LOW,  B=HIGH (01) = ON mode (manual/priming)
// A=HIGH, B=LOW  (10) = AUTO mode
// A=LOW,  B=LOW  (00) = Reserved/unused

// Relay Outputs
#define PUMP_RELAY  D0       // Relay 1 - Pump control
#define FAULT_LIGHT D1       // Relay 2 - Fault indicator light

// Status LEDs (front panel) - mapped to Opta status LEDs
#define LED_LOW_PRESSURE  LED_D0   // LED 1 - Low pressure fault
#define LED_OVERCURRENT   LED_D1   // LED 2 - Overcurrent fault
#define LED_UNDERCURRENT  LED_D2   // LED 3 - Undercurrent fault
#define LED_SYSTEM_OK     LED_D3   // LED 4 - System OK (no fault)

// ============== OPERATION MODES ==============

enum OperationMode {
  MODE_OFF,   // Pump off, all faults ignored
  MODE_ON,    // Pump on (manual), ignores low pressure (for priming), respects current limits
  MODE_AUTO   // Normal automatic operation with all fault protections
};

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

OperationMode operationMode = MODE_AUTO;  // Default to AUTO mode
bool pumpState = false;
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
unsigned long lastModeRead = 0;
unsigned long lastEthCheck = 0;
bool lastButtonState = LOW;
bool ethConnected = false;
bool mqttConfigured = false;

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

  // Initialize status LEDs
  pinMode(LED_LOW_PRESSURE, OUTPUT);
  pinMode(LED_OVERCURRENT, OUTPUT);
  pinMode(LED_UNDERCURRENT, OUTPUT);
  pinMode(LED_SYSTEM_OK, OUTPUT);
  updateStatusLEDs();  // Set initial LED state

  // Initialize button inputs
  pinMode(BTN_USER, INPUT);                 // Built-in USER button
  pinMode(MODE_SELECT_A, INPUT);            // I7 - external pull-up required
  pinMode(MODE_SELECT_B, INPUT);            // I8 - external pull-up required

  // Configure analog resolution
  analogReadResolution(12);

  // Initialize Ethernet & MQTT
  initEthernet();
  initMQTT();

  Serial.println("System ready (non-blocking network).");
  Serial.print("Operation mode: ");
  Serial.println(getModeString(operationMode));
  Serial.print("Low pressure threshold: ");
  Serial.print(LOW_PRESSURE_THRESHOLD);
  Serial.println(" PSI (AUTO mode only)");
  Serial.print("High pressure cutout: ");
  Serial.print(HIGH_PRESSURE_CUTOUT);
  Serial.println(" PSI (ON mode)");
  Serial.print("Overcurrent threshold: ");
  Serial.print(OVERCURRENT_THRESHOLD);
  Serial.print(" A for ");
  Serial.print(OVERCURRENT_DELAY / 1000);
  Serial.println(" seconds");
  Serial.print("Undercurrent threshold: ");
  Serial.print(UNDERCURRENT_THRESHOLD);
  Serial.print(" A for ");
  Serial.print(UNDERCURRENT_DELAY / 1000);
  Serial.println(" seconds (AUTO mode only)");
  Serial.println("Local control active - network connects in background.");
}

// ============== MAIN LOOP ==============

void loop() {
  // === CRITICAL: Local control always runs first (non-blocking) ===

  // Read sensors
  readSensors();

  // Check for mode selector changes
  checkModeSelector();

  // Check for reset button press
  checkResetButton();

  // Check for fault conditions
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
  mqttClient.subscribe(topicReset);
  mqttClient.subscribe(topicModeCommand);
  Serial.println("Subscribed to reset and mode topics");

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

  // Handle mode change commands
  if (topic == topicModeCommand) {
    if (message == "OFF" || message == "0") {
      setOperationMode(MODE_OFF);
    } else if (message == "ON" || message == "1" || message == "MANUAL") {
      setOperationMode(MODE_ON);
    } else if (message == "AUTO" || message == "2" || message == "AUTOMATIC") {
      setOperationMode(MODE_AUTO);
    }
    return;
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

  // Create device object (reused for all entities)
  JSONVar device;
  device["ids"][0] = deviceId;
  device["name"] = deviceName;
  device["mf"] = manufacturer;
  device["mdl"] = model;

  // --- Pressure Sensor ---
  JSONVar pressureConfig;
  pressureConfig["name"] = "Water Pressure";
  pressureConfig["uniq_id"] = String(deviceId) + "_pressure";
  pressureConfig["stat_t"] = topicPressure;
  pressureConfig["unit_of_meas"] = "PSI";
  pressureConfig["dev_cla"] = "pressure";
  pressureConfig["stat_cla"] = "measurement";
  pressureConfig["icon"] = "mdi:gauge";
  pressureConfig["avty_t"] = topicAvailability;
  pressureConfig["pl_avail"] = "online";
  pressureConfig["pl_not_avail"] = "offline";
  pressureConfig["dev"] = device;

  mqttClient.beginMessage("homeassistant/sensor/opta_pump_pressure/config", true, 1);
  mqttClient.print(JSON.stringify(pressureConfig));
  mqttClient.endMessage();

  // --- Amps Sensor ---
  JSONVar ampsConfig;
  ampsConfig["name"] = "Pump Current";
  ampsConfig["uniq_id"] = String(deviceId) + "_amps";
  ampsConfig["stat_t"] = topicAmps;
  ampsConfig["unit_of_meas"] = "A";
  ampsConfig["dev_cla"] = "current";
  ampsConfig["stat_cla"] = "measurement";
  ampsConfig["icon"] = "mdi:current-ac";
  ampsConfig["avty_t"] = topicAvailability;
  ampsConfig["pl_avail"] = "online";
  ampsConfig["pl_not_avail"] = "offline";
  ampsConfig["dev"] = device;

  mqttClient.beginMessage("homeassistant/sensor/opta_pump_amps/config", true, 1);
  mqttClient.print(JSON.stringify(ampsConfig));
  mqttClient.endMessage();

  // --- Pump Status Sensor (read-only) ---
  JSONVar statusConfig;
  statusConfig["name"] = "Water Pump Status";
  statusConfig["uniq_id"] = String(deviceId) + "_status";
  statusConfig["stat_t"] = topicStatus;
  statusConfig["icon"] = "mdi:water-pump";
  statusConfig["avty_t"] = topicAvailability;
  statusConfig["pl_avail"] = "online";
  statusConfig["pl_not_avail"] = "offline";
  statusConfig["dev"] = device;

  mqttClient.beginMessage("homeassistant/binary_sensor/opta_pump_status/config", true, 1);
  mqttClient.print(JSON.stringify(statusConfig));
  mqttClient.endMessage();

  // --- Fault Status Sensor ---
  JSONVar faultConfig;
  faultConfig["name"] = "Pump Fault";
  faultConfig["uniq_id"] = String(deviceId) + "_fault";
  faultConfig["stat_t"] = topicFault;
  faultConfig["icon"] = "mdi:alert-circle";
  faultConfig["avty_t"] = topicAvailability;
  faultConfig["pl_avail"] = "online";
  faultConfig["pl_not_avail"] = "offline";
  faultConfig["dev"] = device;

  mqttClient.beginMessage("homeassistant/sensor/opta_pump_fault/config", true, 1);
  mqttClient.print(JSON.stringify(faultConfig));
  mqttClient.endMessage();

  // --- Reset Button (as a button entity in HA) ---
  JSONVar resetConfig;
  resetConfig["name"] = "Pump Reset";
  resetConfig["uniq_id"] = String(deviceId) + "_reset";
  resetConfig["cmd_t"] = topicReset;
  resetConfig["pl_prs"] = "RESET";
  resetConfig["icon"] = "mdi:restart";
  resetConfig["avty_t"] = topicAvailability;
  resetConfig["pl_avail"] = "online";
  resetConfig["pl_not_avail"] = "offline";
  resetConfig["dev"] = device;

  mqttClient.beginMessage("homeassistant/button/opta_pump_reset/config", true, 1);
  mqttClient.print(JSON.stringify(resetConfig));
  mqttClient.endMessage();

  // --- Mode Select (OFF/ON/AUTO) ---
  JSONVar modeConfig;
  modeConfig["name"] = "Pump Mode";
  modeConfig["uniq_id"] = String(deviceId) + "_mode";
  modeConfig["stat_t"] = topicMode;
  modeConfig["cmd_t"] = topicModeCommand;
  modeConfig["options"][0] = "OFF";
  modeConfig["options"][1] = "ON";
  modeConfig["options"][2] = "AUTO";
  modeConfig["icon"] = "mdi:toggle-switch";
  modeConfig["avty_t"] = topicAvailability;
  modeConfig["pl_avail"] = "online";
  modeConfig["pl_not_avail"] = "offline";
  modeConfig["dev"] = device;

  mqttClient.beginMessage("homeassistant/select/opta_pump_mode/config", true, 1);
  mqttClient.print(JSON.stringify(modeConfig));
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

  // Read USER button (active HIGH when pressed on Opta)
  bool buttonState = digitalRead(BTN_USER);

  // Detect rising edge (button press)
  if (lastButtonState == LOW && buttonState == HIGH) {
    Serial.println("USER button pressed - resetting fault");
    resetFault();
  }

  lastButtonState = buttonState;
}

// ============== MODE SELECTOR ==============

void checkModeSelector() {
  unsigned long now = millis();
  if (now - lastModeRead < DEBOUNCE_DELAY) return;
  lastModeRead = now;

  // Read mode selector pins (active LOW with external pull-ups)
  bool pinA = digitalRead(MODE_SELECT_A);
  bool pinB = digitalRead(MODE_SELECT_B);

  // Determine mode from truth table
  OperationMode newMode;
  
  if (pinA == HIGH && pinB == HIGH) {
    // 11 = OFF mode
    newMode = MODE_OFF;
  } else if (pinA == LOW && pinB == HIGH) {
    // 01 = ON mode (manual/priming)
    newMode = MODE_ON;
  } else if (pinA == HIGH && pinB == LOW) {
    // 10 = AUTO mode
    newMode = MODE_AUTO;
  } else {
    // 00 = Reserved/unused, default to OFF for safety
    newMode = MODE_OFF;
  }

  // Only change mode if different
  if (newMode != operationMode) {
    Serial.print("Mode selector changed: ");
    Serial.print(pinA ? "1" : "0");
    Serial.print(pinB ? "1" : "0");
    Serial.print(" -> ");
    setOperationMode(newMode);
  }
}

// ============== OPERATION MODE CONTROL ==============

void setOperationMode(OperationMode newMode) {
  if (operationMode == newMode) return;
  
  operationMode = newMode;
  Serial.print("Mode changed to: ");
  Serial.println(getModeString(operationMode));
  
  // Clear fault when cycling to OFF mode
  if (newMode == MODE_OFF && faultState != FAULT_NONE) {
    Serial.println("Cycling to OFF mode - clearing fault");
    resetFault();
  }
  
  // Update pump state based on new mode
  updatePumpOutput();
  
  // Publish mode change
  publishModeStatus();
}

const char* getModeString(OperationMode mode) {
  switch (mode) {
    case MODE_OFF:  return "OFF";
    case MODE_ON:   return "ON";
    case MODE_AUTO: return "AUTO";
    default:        return "UNKNOWN";
  }
}

// ============== FAULT HANDLING ==============

void checkFaults() {
  // No fault checking in OFF mode
  if (operationMode == MODE_OFF) return;
  
  // Skip if already faulted
  if (faultState != FAULT_NONE) return;

  // === ON MODE (Manual/Priming) ===
  if (operationMode == MODE_ON) {
    // Check HIGH PRESSURE CUTOUT - stop pump if pressure too high
    if (pumpState && currentPressure >= HIGH_PRESSURE_CUTOUT) {
      pumpState = false;
      digitalWrite(PUMP_RELAY, LOW);
      Serial.print("High pressure cutout: ");
      Serial.print(currentPressure);
      Serial.println(" PSI");
      publishPumpStatus();
      // Not a fault - just a cutout, pump can restart when pressure drops
      return;
    }
    
    // Check OVERCURRENT only
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
    
    // Ignore undercurrent in ON mode
    if (undercurrentTiming) {
      Serial.println("Undercurrent check disabled in ON mode");
      undercurrentTiming = false;
    }
    
    return;  // Skip AUTO mode checks
  }

  // === AUTO MODE (Full fault protection) ===
  
  // LOW PRESSURE: Only check in AUTO mode when pump is running
  if (pumpState && currentPressure < LOW_PRESSURE_THRESHOLD) {
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

  // UNDERCURRENT: Only check in AUTO mode when pump is running
  if (pumpState && currentAmps <= UNDERCURRENT_THRESHOLD) {
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
  // Light up the appropriate LED based on fault state
  // Turn off all LEDs first, then turn on the correct one
  digitalWrite(LED_LOW_PRESSURE, (faultState == FAULT_LOW_PRESSURE) ? HIGH : LOW);
  digitalWrite(LED_OVERCURRENT, (faultState == FAULT_OVERCURRENT) ? HIGH : LOW);
  digitalWrite(LED_UNDERCURRENT, (faultState == FAULT_UNDERCURRENT) ? HIGH : LOW);
  digitalWrite(LED_SYSTEM_OK, (faultState == FAULT_NONE) ? HIGH : LOW);
}

// ============== PUMP CONTROL ==============

void updatePumpOutput() {
  bool shouldRun = false;
  
  switch (operationMode) {
    case MODE_OFF:
      // Pump always off in OFF mode
      shouldRun = false;
      break;
      
    case MODE_ON:
      // Pump always on in ON mode (manual/priming), unless faulted
      shouldRun = (faultState == FAULT_NONE);
      break;
      
    case MODE_AUTO:
      // Pump on in AUTO mode, unless faulted
      shouldRun = (faultState == FAULT_NONE);
      break;
  }

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
  Serial.print(" | Mode: ");
  Serial.print(getModeString(operationMode));
  Serial.print(" | Pump: ");
  Serial.print(pumpState ? "ON" : "OFF");
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
  publishModeStatus();
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

void publishModeStatus() {
  if (!mqttClient.connected()) return;

  mqttClient.beginMessage(topicMode, true, 1);
  mqttClient.print(getModeString(operationMode));
  mqttClient.endMessage();
}