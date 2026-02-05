# Arduino Opta Water Pump Controller

Industrial-grade water pump controller with comprehensive monitoring, fault protection, and Home Assistant integration via MQTT.

## Features

- **Real-time Monitoring**: 0-10V pressure sensor (0-87 PSI) and 0-20A current sensor
- **Multi-Mode Operation**: OFF, ON (manual/priming), and AUTO modes
- **Advanced Fault Protection**: Low/high pressure, overcurrent, and undercurrent detection with configurable delays
- **MQTT Integration**: Full Home Assistant auto-discovery with retained messages
- **Non-blocking Architecture**: Local control operates independently of network status
- **Visual Feedback**: 4 front panel LEDs for fault indication and system status
- **Physical Controls**: Built-in USER button for fault reset and 3-position mode selector
- **Graceful Recovery**: 6-second pressure grace period after fault reset to prevent false triggers

## Hardware Requirements

### Arduino Opta
- **Board**: Arduino Opta (Mbed OS core, NOT Zephyr)
- **Ethernet**: Built-in Ethernet port

### Inputs
- **I1 (A0)**: 0-10V pressure sensor (0-87 PSI range)
- **I2 (A1)**: 0-10V current sensor (0-20A range)
- **BTN_USER**: Built-in USER button for fault reset (active HIGH)
- **I7 (A6)**: Mode selector Pin A (requires external 10K pull-up resistor to +V)
- **I8 (A7)**: Mode selector Pin B (requires external 10K pull-up resistor to +V)

### Outputs
- **Relay 1 (D0)**: Pump control relay
- **Relay 2 (D1)**: Fault indicator light

### Status LEDs (Front Panel)
- **LED 1**: Low pressure fault indicator
- **LED 2**: Overcurrent fault indicator
- **LED 3**: Undercurrent fault indicator
- **LED 4**: System OK (no faults)

## Installation

### 1. Install Arduino IDE Board Support

**IMPORTANT**: This code requires the Mbed OS core, not the newer Zephyr core.

1. Open Arduino IDE
2. Go to **Tools > Board > Boards Manager**
3. Search for "Mbed OS Opta"
4. Install **Arduino Mbed OS Opta Boards** (NOT the Zephyr version)
5. Select **Tools > Board > Arduino Mbed OS Opta Boards > Arduino Opta**

### 2. Install Required Libraries

Via Library Manager (**Sketch > Include Library > Manage Libraries**):
- `ArduinoMqttClient` (by Arduino)
- `Arduino_JSON` (by Arduino) - used for Home Assistant discovery

Built-in libraries (no installation needed):
- `Ethernet` (included with Mbed OS Opta core)

### 3. Configure Network Settings

Edit the configuration section in the code:

```cpp
// Network Settings
const byte mac[] = { 0xA8, 0x61, 0x0A, 0x50, 0xA6, 0xB7 };  // Unique MAC address
const IPAddress ip(10, 58, 82, 99);           // Opta static IP address
const IPAddress gateway(10, 58, 82, 1);       // Your network gateway
const IPAddress subnet(255, 255, 255, 0);     // Subnet mask

// MQTT Settings
const char* mqttBroker = "10.58.82.15";       // MQTT broker IP address
const int mqttPort = 1883;                    // MQTT port (default 1883)
const char* mqttUser = "your-username";       // MQTT username (if required)
const char* mqttPass = "your-password";       // MQTT password (if required)
```

**Note**: Use a unique MAC address for each Opta device on your network.

### 4. Adjust Fault Thresholds (Optional)

Customize protection thresholds based on your pump specifications:

```cpp
// Pressure Thresholds
const float LOW_PRESSURE_THRESHOLD = 30.0;     // PSI - fault if below (AUTO mode only)
const float HIGH_PRESSURE_CUTOUT = 60.0;       // PSI - auto cutout (ON mode only)

// Current Thresholds
const float OVERCURRENT_THRESHOLD = 10.0;      // Amps - fault if exceeded
const float UNDERCURRENT_THRESHOLD = 3.0;      // Amps - fault if below (AUTO mode only)

// Fault Delays (prevent false triggers)
const unsigned long OVERCURRENT_DELAY = 6000;  // ms - 6 seconds
const unsigned long UNDERCURRENT_DELAY = 6000; // ms - 6 seconds
const unsigned long PRESSURE_GRACE_PERIOD = 6000; // ms - grace period after reset
```

**Tip**: Increase delay times if you experience false fault triggers during normal operation.

## Operation

### Physical Control Panel Wiring

#### Reset Button (Built-in USER Button)
The Arduino Opta has a built-in USER button that serves as the fault reset:
- No external wiring required
- Press to clear any fault condition
- Works in all operation modes
- Active HIGH detection with debouncing

#### Mode Selector (I7 / A6 and I8 / A7)

**CRITICAL: External 10K pull-up resistors are mandatory!** The Opta's analog input pins (A6, A7) do not support internal pull-ups.

The mode selector uses 2 digital inputs with a binary encoding scheme:

**Pin State Truth Table (Active LOW):**

| Position | Pin A (I7) | Pin B (I8) | Binary | Mode |
|----------|------------|------------|--------|------|
| 1 | HIGH | HIGH | 11 | OFF |
| 2 | LOW | HIGH | 01 | ON (Manual/Priming) |
| 3 | HIGH | LOW | 10 | AUTO |
| Reserved | LOW | LOW | 00 | OFF (safety default) |

**Required Components:**
- Two 10K ohm resistors (pull-up resistors)
- 3-position rotary switch OR two SPST toggle switches

**Wiring Option 1: 3-Position Rotary Switch (Recommended)**

```
     +V (use Opta's +24V, +5V, or external +3.3V)
      |           |
     10K         10K  ← Pull-up resistors (1/4W or higher)
      |           |
   I7 (A6)    I8 (A7)
      |           |
      └─────┬─────┘
            │
      Rotary Switch
            │
          GND

Position 1: No connection    → Both HIGH (11) → OFF mode
Position 2: Connect to I7    → A LOW, B HIGH (01) → ON mode  
Position 3: Connect to I8    → A HIGH, B LOW (10) → AUTO mode
```

**Installation Steps:**
1. Install 10K ohm resistors (1/4W or higher) from I7 to +V and I8 to +V
2. Connect rotary switch common terminal to GND
3. Connect position terminals to I7 and I8 (one position left open for OFF mode)
4. Verify voltage at I7/I8 reads ~3.3V when switch is open (HIGH state)
5. Verify voltage drops to ~0V when switch connects pin to GND (LOW state)

**Wiring Option 2: Two SPST Toggle Switches**

```
     +V              +V
      |               |
     10K             10K  ← Pull-up resistors
      |               |
   I7 (A6)        I8 (A7)
      |               |
   Switch A        Switch B
      |               |
     GND             GND

Switch States:
Both OFF (HIGH/HIGH) → OFF mode
Switch A ON (LOW/HIGH) → ON mode
Switch B ON (HIGH/LOW) → AUTO mode
Both ON (LOW/LOW) → Reserved (defaults to OFF for safety)
```

**Important Notes:**
- **10K external pull-up resistors are mandatory** - Opta analog inputs lack internal pull-ups
- Use Opta's +24V output or external +5V/+3.3V for pull-up voltage source
- Mode changes are debounced (50ms) and published to MQTT automatically
- Mode can also be changed remotely via MQTT (topic: `opta/pump/mode/set`)
- Current mode is always visible in Home Assistant and serial monitor
- Physical switch overrides MQTT commands (hardware priority)

### Operation Modes

The controller has three operation modes:

#### OFF Mode
- Pump is completely disabled
- All fault checking suspended
- Cycling to OFF mode automatically clears any active faults
- Use for maintenance, system shutdown, or when pump is not needed

#### ON Mode (Manual/Priming)
- Pump runs continuously (manual override)
- **Ignores low pressure warnings** - ideal for priming or filling system
- **High pressure cutout active** - automatically stops at 60 PSI, auto-restarts when pressure drops below threshold
- **Ignores undercurrent faults** - accommodates variable loads during priming
- **Overcurrent protection active** - prevents motor damage (cannot be bypassed)
- Use for initial system priming, manual operation, or troubleshooting

#### AUTO Mode (Default)
- Normal automatic operation with full protection
- All fault checks active: low pressure, high pressure, overcurrent, undercurrent
- Pump starts automatically when mode is selected (if no faults present)
- 6-second grace period after fault reset prevents false low-pressure triggers
- Recommended for standard unattended operation

### Normal Operation

1. **Power On**: System initializes, connects to network (non-blocking)
2. **Select Mode**: Use physical mode selector or MQTT command
3. **AUTO Mode**: Pump starts automatically with full fault protection
4. **ON Mode**: Pump runs continuously (manual/priming mode)
5. **Monitoring**: System continuously reads sensors and checks for faults
6. **Status Updates**: Data published to MQTT every 1 second, serial output for local monitoring
7. **Fault Handling**: System automatically stops pump and activates fault indicator on any fault condition

### Fault Protection

#### Low Pressure Fault
- **Trigger**: Pressure < 30 PSI while pump is running (AUTO mode only)
- **Delay**: Immediate (with 6-second grace period after fault reset)
- **Action**: Pump stops immediately, fault light activates, LED 1 illuminates
- **Recovery**: Press reset button or cycle to OFF mode, then return to AUTO
- **Bypass**: Switch to ON mode to bypass low pressure check during priming
- **Common Causes**: Dry well, closed valve, leak in suction line, air in system

#### High Pressure Cutout
- **Trigger**: Pressure ≥ 60 PSI while pump is running (ON mode only)
- **Delay**: Immediate
- **Action**: Pump stops temporarily, automatically restarts when pressure drops
- **Recovery**: Automatic - no reset required (not a fault condition)
- **LED**: No fault LED illuminates (normal operation)
- **Note**: Prevents over-pressurization during manual priming

#### Overcurrent Fault
- **Trigger**: Current > 10A sustained for 6+ seconds (ON and AUTO modes)
- **Delay**: 6 seconds (prevents false triggers from motor startup surge)
- **Action**: Pump stops, fault light activates, LED 2 illuminates
- **Recovery**: Press reset button or cycle to OFF mode (investigate cause before restart)
- **Bypass**: Cannot be bypassed - critical safety protection
- **Common Causes**: Seized pump, shorted motor, mechanical binding, voltage issues

#### Undercurrent Fault
- **Trigger**: Current ≤ 3A sustained for 6+ seconds while running (AUTO mode only)
- **Delay**: 6 seconds (allows for normal load variations)
- **Action**: Pump stops, fault light activates, LED 3 illuminates
- **Recovery**: Press reset button or cycle to OFF mode
- **Bypass**: Switch to ON mode for manual operation (ignores undercurrent)
- **Common Causes**: Dry run (no water), broken impeller, open circuit, sensor malfunction

### Manual Controls

#### Reset Button (Built-in USER Button)
- Press to clear any active fault condition
- 50ms debounce prevents accidental double-triggers
- Initiates 6-second pressure grace period (prevents immediate re-fault)
- Post-reset behavior depends on current mode:
  - **OFF mode**: Pump remains off, fault cleared
  - **ON mode**: Pump restarts immediately (if no other faults)
  - **AUTO mode**: Pump restarts immediately (if no other faults)
- Reset action is logged to serial monitor and published to MQTT

#### Mode Selector (I7 + I8)
- Physical 3-position switch (or two toggle switches) for mode selection
- Changes detected every 50ms (debounced)
- Mode changes take effect immediately
- Cycling to OFF mode automatically clears any active fault
- Current mode published to MQTT topic `opta/pump/mode`
- Mode visible in Home Assistant and serial monitor
- Can also be changed remotely via MQTT topic `opta/pump/mode/set`
- Physical switch has priority over MQTT commands

## MQTT Topics

### Published Topics

| Topic | Description | Example |
|-------|-------------|---------|
| `opta/pump/pressure` | Current pressure in PSI | `45.2` |
| `opta/pump/amps` | Current draw in amps | `7.35` |
| `opta/pump/status` | Pump state | `ON` or `OFF` |
| `opta/pump/fault` | Fault status | `OK`, `LOW_PRESSURE`, `OVERCURRENT`, `UNDERCURRENT` |
| `opta/pump/mode` | Operation mode | `OFF`, `ON`, `AUTO` |
| `opta/pump/availability` | Device availability | `online` or `offline` |

### Subscribed Topics

| Topic | Command | Description |
|-------|---------|-------------|
| `opta/pump/reset` | `RESET` | Clear fault condition |
| `opta/pump/mode/set` | `OFF`, `ON`, `AUTO` | Change operation mode |

## Home Assistant Integration

The controller automatically configures Home Assistant entities via MQTT discovery. Entities appear automatically when the device connects - no manual YAML configuration required.

### Auto-Discovered Entities

| Entity Type | Name | Description | Unit | Device Class |
|-------------|------|-------------|------|--------------|
| Sensor | Water Pressure | Real-time pressure reading | PSI | pressure |
| Sensor | Pump Current | Real-time current draw | A | current |
| Binary Sensor | Water Pump Status | Pump ON/OFF state (read-only) | - | - |
| Sensor | Pump Fault | Current fault status | - | - |
| Sensor | Pump Mode | Current operation mode | - | - |
| Button | Pump Reset | Clear fault condition | - | - |

### Entity Features

- **Availability Tracking**: All entities show "unavailable" if device goes offline
- **Retained Messages**: Status values retained on broker for immediate state on HA restart
- **Device Grouping**: All entities grouped under "Water Pump Controller" device
- **Icons**: Appropriate MDI icons assigned (gauge, current-ac, water-pump, alert-circle, etc.)
- **State Classes**: Sensors marked as "measurement" for proper history tracking

### Discovery Topics

Discovery configs published to `homeassistant/[entity_type]/opta_pump_[entity]/config`

No manual configuration needed - entities appear automatically when the device connects.

## Serial Monitor Output

Connect at 115200 baud to see real-time status:

```
P: 45.2 PSI | A: 7.35 | Mode: AUTO | Pump: ON | Fault: OK | Net: MQTT
```

Status indicators:
- **P**: Pressure reading
- **A**: Current reading
- **Mode**: Current operation mode (OFF/ON/AUTO)
- **Pump**: ON/OFF state
- **Fault**: Current fault state
- **Net**: Network status (MQTT/ETH/DOWN)

## Troubleshooting

### Ethernet Not Connecting
- **Check cable**: Verify Ethernet cable is securely connected and not damaged
- **IP conflict**: Ensure IP address doesn't conflict with other devices (ping test)
- **Network settings**: Verify gateway and subnet match your network configuration
- **Link status**: Check serial monitor for "Ethernet connected" message
- **LED indicator**: Opta Ethernet port has link/activity LEDs
- **Non-blocking**: System continues local control even without network

### MQTT Not Connecting
- **Broker status**: Verify MQTT broker is running (`mosquitto -v` or check service status)
- **Network first**: MQTT requires Ethernet connection - check "Net: ETH" in serial output
- **Credentials**: Verify username/password if authentication is enabled
- **Port**: Confirm broker is listening on port 1883 (or configured port)
- **Firewall**: Check firewall rules allow MQTT traffic
- **Timeout**: Connection attempts timeout after 2 seconds (non-blocking)
- **Serial output**: Watch for "MQTT connecting..." and error codes

### Pump Won't Start
- **Mode check**: Verify mode is ON or AUTO (not OFF) - check serial monitor
- **Fault active**: Look for lit fault LEDs (LED 1, 2, or 3)
- **Reset**: Press USER button to clear any fault
- **Mode selector**: Verify pull-up resistors installed and switch wiring correct
- **Voltage check**: Measure I7/I8 pins - should read ~3.3V when HIGH, ~0V when LOW
- **Serial debug**: Monitor serial output for mode changes and fault messages
- **Relay test**: Listen for relay click when pump should activate

### False Fault Triggers
- **Increase delays**: Raise `OVERCURRENT_DELAY` or `UNDERCURRENT_DELAY` to 10-15 seconds
- **Adjust thresholds**: Lower `OVERCURRENT_THRESHOLD` or raise `UNDERCURRENT_THRESHOLD`
- **Sensor calibration**: Verify sensor output matches actual pressure/current
- **Wiring check**: Inspect sensor wiring for noise, loose connections, or ground loops
- **Grace period**: 6-second pressure grace period after reset prevents immediate re-fault
- **Startup surge**: Overcurrent delay accounts for motor inrush current

### Mode Selector Not Working
- **Pull-up resistors**: Verify 10K resistors installed from I7/I8 to +V
- **Voltage test**: Measure pins with multimeter - should toggle between 0V and 3.3V
- **Switch type**: Ensure switch grounds pins (active LOW logic)
- **Debounce**: Mode changes debounced every 50ms - may have slight delay
- **Serial monitor**: Watch for "Mode changed to:" messages

### Sensor Readings Incorrect
- **Calibration**: Adjust `PRESSURE_MAX`, `PRESSURE_MIN`, `AMP_MAX`, `AMP_MIN` constants
- **Voltage check**: Verify sensor outputs 0-10V range with multimeter
- **Resolution**: Opta uses 12-bit ADC (4096 steps) for high accuracy
- **Wiring**: Check for loose connections, incorrect polarity, or voltage drops
- **Sensor specs**: Confirm sensor output matches expected 0-10V range

## Sensor Calibration

The default calibration assumes linear 0-10V sensors:
- **Pressure sensor**: 0-10V = 0-87 PSI
- **Current sensor**: 0-10V = 0-20A

To adjust calibration for your specific sensors, modify these constants:

```cpp
// Sensor Calibration
const float PRESSURE_MAX = 87.0;   // Maximum pressure in PSI at 10V
const float PRESSURE_MIN = 0.0;    // Minimum pressure in PSI at 0V
const float AMP_MAX = 20.0;        // Maximum current in amps at 10V
const float AMP_MIN = 0.0;         // Minimum current in amps at 0V
```

**Calibration Procedure:**
1. Measure actual sensor voltage output with multimeter
2. Compare to known pressure/current values
3. Calculate scaling factors
4. Update constants in code
5. Verify readings in serial monitor match actual values

## Sensor Calibration

Default calibration assumes:
- Pressure sensor: 0-10V = 0-100 PSI
- Current sensor: 0-10V = 0-20A

To adjust, modify these constants:

```cpp
const float PRESSURE_MAX = 100.0;
const float PRESSURE_MIN = 0.0;
const float AMP_MAX = 20.0;
const float AMP_MIN = 0.0;
```

## Safety Notes

- Overcurrent protection cannot be bypassed for safety reasons
- Always verify proper sensor calibration before operation
- Test fault conditions before deploying in production
- Ensure proper electrical isolation for sensors and relays
- Follow local electrical codes for pump installation

## License

This project is provided as-is for educational and industrial automation purposes.
