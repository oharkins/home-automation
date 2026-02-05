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
- **I4 (A3)**: Mode switch input (optional, for future physical mode switch)

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
- Pump controlled via MQTT commands
- Standard operating mode for normal use

### Normal Operation

1. System starts in AUTO mode with pump OFF
2. Send MQTT command or use Home Assistant to turn pump ON (in AUTO mode)
3. System monitors pressure and current continuously
4. Status published to MQTT every 2 seconds
5. Switch to ON mode for priming (bypasses low pressure fault)

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
- Pump will remain OFF after reset in AUTO mode
- In ON mode, pump will restart after fault is cleared

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
| `opta/pump/command` | `ON`, `OFF`, `TOGGLE` | Control pump (AUTO mode only) |
| `opta/pump/reset` | `RESET` | Clear fault condition |
| `opta/pump/mode/set` | `OFF`, `ON`, `AUTO` | Change operation mode |

## Home Assistant Integration

The controller automatically configures Home Assistant entities via MQTT discovery:

- **Select**: Pump Mode (OFF/ON/AUTO)
- **Sensor**: Water Pressure (PSI)
- **Sensor**: Pump Current (A)
- **Switch**: Water Pump (ON/OFF control in AUTO mode)
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
- Check for active fault (LED 1, 2, or 3 lit)
- Press reset button to clear fault
- Verify MQTT command is being received (check serial monitor)

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
