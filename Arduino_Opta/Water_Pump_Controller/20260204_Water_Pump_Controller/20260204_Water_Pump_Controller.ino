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
 * Operation Modes:
 * - OFF:  Pump is completely off, all fault checking disabled
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
 * - Reset via physical button on I3 (A2) or MQTT reset command
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
 */

#include <Ethernet.h>
#include <ArduinoMqttClient.h>

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
#define RESET_BUTTON_PIN A2  // I3 - Reset button (active LOW with internal pull-up)
#define MODE_SWITCH_PIN  A3  // I4 - Mode switch input (for physical toggle if needed)

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
unsigned long lastEthCheck = 0;
bool lastButtonState = HIGH;
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

  // Initialize button inputs (active LOW with internal pull-up)
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);  // Optional physical mode switch

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

  // --- Pump Status Sensor (read-only) ---
  String statusConfig = String("{") +
    "\"name\":\"Water Pump Status\"," +
    "\"uniq_id\":\"" + deviceId + "_status\"," +
    "\"stat_t\":\"" + topicStatus + "\"," +
    "\"icon\":\"mdi:water-pump\"," +
    availabilityJson + "," +
    deviceJson +
    "}";

  mqttClient.beginMessage("homeassistant/binary_sensor/opta_pump_status/config", true, 1);
  mqttClient.print(statusConfig);
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

  // --- Mode Select (OFF/ON/AUTO) ---
  String modeConfig = String("{") +
    "\"name\":\"Pump Mode\"," +
    "\"uniq_id\":\"" + deviceId + "_mode\"," +
    "\"stat_t\":\"" + topicMode + "\"," +
    "\"cmd_t\":\"" + topicModeCommand + "\"," +
    "\"options\":[\"OFF\",\"ON\",\"AUTO\"]," +
    "\"icon\":\"mdi:toggle-switch\"," +
    availabilityJson + "," +
    deviceJson +
    "}";

  mqttClient.beginMessage("homeassistant/select/opta_pump_mode/config", true, 1);
  mqttClient.print(modeConfig);
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

// ============== OPERATION MODE CONTROL ==============

void setOperationMode(OperationMode newMode) {
  if (operationMode == newMode) return;
  
  operationMode = newMode;
  Serial.print("Mode changed to: ");
  Serial.println(getModeString(operationMode));
  
  // Clear fault when changing modes
  if (faultState != FAULT_NONE) {
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