# Arduino Opta Water Pump Controller

Industrial water pump controller with pressure monitoring, current sensing, fault protection, and Home Assistant integration via MQTT.

## Features

- **Pressure Monitoring**: 0-10V pressure sensor (0-100 PSI)
- **Current Sensing**: 0-10V/0-20A current sensor
- **Fault Protection**: Low pressure, overcurrent, and undercurrent detection
- **MQTT Integration**: Full Home Assistant auto-discovery support
- **Non-blocking Network**: Local control works even when network is down
- **Front Panel Status LEDs**: Visual fault indication
- **Manual Controls**: Physical reset and jog/bypass buttons

## Hardware Requirements

### Arduino Opta
- **Board**: Arduino Opta (Mbed OS core, NOT Zephyr)
- **Ethernet**: Built-in Ethernet port

### Inputs
- **I1 (A0)**: 0-10V pressure sensor
- **I2 (A1)**: 0-10V current sensor (0-20A)
- **I3 (A2)**: Reset button (active LOW with internal pull-up)
- **I7 (A6)**: Mode selector - Pin A (requires external 10K pull-up resistor)
- **I8 (A7)**: Mode selector - Pin B (requires external 10K pull-up resistor)

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
- `ArduinoMqttClient`

Built-in libraries (no installation needed):
- `Ethernet`

### 3. Configure Network Settings

Edit the following in the code:

```cpp
// Network Settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 100);        // Opta IP address
IPAddress gateway(192, 168, 1, 1);     // Your gateway
IPAddress subnet(255, 255, 255, 0);

// MQTT Settings
const char* mqttBroker = "192.168.1.50";  // Your MQTT broker IP
const int mqttPort = 1883;
const char* mqttUser = "";                 // Optional
const char* mqttPass = "";                 // Optional
```

### 4. Adjust Fault Thresholds (Optional)

```cpp
const float LOW_PRESSURE_THRESHOLD = 30.0;    // PSI (AUTO mode)
const float HIGH_PRESSURE_CUTOUT = 60.0;      // PSI (ON mode cutout)
const float OVERCURRENT_THRESHOLD = 10.0;     // Amps
const float UNDERCURRENT_THRESHOLD = 3.0;     // Amps (AUTO mode)
const unsigned long UNDERCURRENT_DELAY = 6000; // ms
const unsigned long OVERCURRENT_DELAY = 6000;  // ms
```

## Operation

### Physical Control Panel Wiring

#### Reset Button (I3 / A2)
Connect a momentary push button between I3 and GND:
- Internal pull-up resistor enabled (active LOW)
- Press to clear any fault condition
- Works in all operation modes

#### Mode Selector (I7 / A6 and I8 / A7)

**IMPORTANT: External pull-up resistors required!** The Opta's analog input pins do not support internal pull-ups.

The mode selector uses 2 digital inputs to create a 3-position switch:

**Pin State Truth Table:**

| Position | Pin A (I7) | Pin B (I8) | Mode |
|----------|------------|------------|------|
| 1 | HIGH | HIGH | OFF |
| 2 | LOW | HIGH | ON (Manual/Priming) |
| 3 | HIGH | LOW | AUTO |

**Required Components:**
- Two 10K ohm resistors (pull-up resistors)
- 3-position rotary switch OR two SPST toggle switches

**Wiring Option 1: 3-Position Rotary Switch (Recommended)**

```
     +V (use +24V, +5V, or +3.3V from Opta)
      |           |
     10K         10K  (pull-up resistors)
      |           |
   I7 (A6)    I8 (A7)
      |           |
Position 1: Open      Open       → OFF mode
Position 2: Ground    Open       → ON mode  
Position 3: Open      Ground     → AUTO mode
      |           |
   Common Ground (GND)
```

Wiring steps:
1. Connect a 10K resistor from I7 to +V (use Opta's +24V output or external +5V/+3.3V)
2. Connect a 10K resistor from I8 to +V
3. Connect the common terminal of the rotary switch to GND
4. Connect one position terminal to I7 (A6)
5. Connect another position terminal to I8 (A7)
6. Leave the third position unconnected (both pins HIGH = OFF)

**Wiring Option 2: Two SPST Toggle Switches**

```
     +V              +V
      |               |
     10K             10K
      |               |
Switch A: I7 (A6) ←→ GND
Switch B: I8 (A7) ←→ GND

Both OFF (HIGH/HIGH) = OFF mode
Switch A ON (LOW/HIGH) = ON mode
Switch B ON (HIGH/LOW) = AUTO mode
(Both ON is reserved/defaults to OFF)
```

**Notes:**
- **External 10K pull-up resistors are required** - Opta analog inputs don't have internal pull-ups
- Use the Opta's +24V output, or external +5V/+3.3V for the pull-up voltage
- Mode changes are debounced and published to MQTT automatically
- Mode can also be changed via MQTT if physical switch is not installed
- Current mode is always displayed in Home Assistant

### Operation Modes

The controller has three operation modes:

#### OFF Mode
- Pump is completely off
- All fault checking disabled
- Use when system is not in use

#### ON Mode (Manual/Priming)
- Pump runs continuously
- **Ignores low pressure warnings** - perfect for priming the pump
- **Respects high pressure cutout** - automatically stops at 60 PSI and restarts when pressure drops
- **Ignores undercurrent faults** - allows for variable loads during priming
- Still protects against overcurrent faults
- Use this mode when you need to prime the pump or run it manually

#### AUTO Mode (Default)
- Normal automatic operation
- Full fault protection active (low pressure, overcurrent, undercurrent)
- Pump turns on automatically when mode is selected
- Standard operating mode for normal use

### Normal Operation

1. Set mode selector to desired position (or use MQTT to change mode)
2. In AUTO mode, pump turns on automatically with full fault protection
3. In ON mode, pump runs continuously (for priming/manual operation)
4. System monitors pressure and current continuously
5. Status published to MQTT every 2 seconds

### Fault Protection

#### Low Pressure Fault
- **Trigger**: Pressure < 30 PSI while pump is running (AUTO mode only)
- **Action**: Pump immediately stops, fault light turns on
- **LED**: LED 1 illuminates
- **Bypass**: Switch to ON mode to bypass low pressure fault for priming

#### High Pressure Cutout
- **Trigger**: Pressure ≥ 60 PSI while pump is running (ON mode only)
- **Action**: Pump stops temporarily, automatically restarts when pressure drops
- **LED**: No fault LED (not a fault condition)
- **Note**: This is a safety cutout, not a fault - no reset needed

#### Overcurrent Fault
- **Trigger**: Current > 10A for more than 6 seconds (ON and AUTO modes)
- **Action**: Pump stops, fault light turns on
- **LED**: LED 2 illuminates
- **Note**: Cannot be bypassed (safety protection)

#### Undercurrent Fault
- **Trigger**: Current ≤ 3A for more than 6 seconds while running (AUTO mode only)
- **Action**: Pump stops, fault light turns on
- **LED**: LED 3 illuminates
- **Bypass**: Switch to ON mode for manual operation (ignores undercurrent)

### Manual Controls

#### Reset Button (I3)
- Press to clear any fault condition
- Pump behavior after reset depends on mode:
  - **OFF mode**: Pump stays off
  - **ON mode**: Pump restarts immediately
  - **AUTO mode**: Pump restarts immediately

#### Mode Selector (I7 + I8)
- Physical 3-position switch to select operation mode
- Changes take effect immediately
- Current mode is published to MQTT and visible in Home Assistant
- Can also be changed remotely via MQTT

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

The controller automatically configures Home Assistant entities via MQTT discovery:

- **Select**: Pump Mode (OFF/ON/AUTO) - control via UI or physical switch
- **Sensor**: Water Pressure (PSI)
- **Sensor**: Pump Current (A)
- **Binary Sensor**: Water Pump Status (read-only, shows ON/OFF state)
- **Sensor**: Pump Fault (status text)
- **Button**: Pump Reset (clear faults)

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
- Check cable connection
- Verify IP address doesn't conflict with other devices
- Ensure gateway and subnet are correct

### MQTT Not Connecting
- Verify broker IP and port
- Check username/password if authentication is enabled
- Ensure broker is running and accessible

### Pump Won't Start
- Check current mode (must be ON or AUTO for pump to run)
- Check for active fault (LED 1, 2, or 3 lit)
- Press reset button to clear fault
- Verify mode selector wiring if using physical switch
- Check serial monitor for mode changes

### False Fault Triggers
- Adjust threshold values in configuration
- Check sensor wiring and calibration
- Increase delay times for overcurrent/undercurrent

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
