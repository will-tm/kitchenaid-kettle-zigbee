# KitchenAid 5KEK1522 Zigbee Kettle Controller

Zigbee Router firmware for custom nRF54L15 hardware to add smart home connectivity to the KitchenAid 5KEK1522 kettle.

## Features

- **Zigbee Router** - Always-on device that extends your Zigbee mesh network
- **On/Off State Reporting** - Reads kettle heating element state from GPIO
- **Target Temperature** - Reads temperature setpoint from analog input (linear potentiometer)
- **Current Temperature** - Reads water temperature from 100K NTC thermistor
- **Status LED** - Blinks when not joined, solid off when joined
- **Pairing Button** - Long press (3s) to factory reset and enter pairing mode
- **OTA Updates** - Over-the-air firmware updates via Zigbee

## Hardware Requirements

- Custom nRF54L15 board (or nRF54L15 DK for development)
- External 32kHz crystal (LFXO)
- GPIO for kettle state input
- ADC channels for temperature sensing
- Push button for pairing
- Status LED

## Wiring Diagram

```
                        nRF54L15
                     +-----------+
                     |           |
    Pairing Button --| P2.02     |-- Via MOSFET level shifter (if 5V)
                     |   (sw0)   |
                     |           |
      Status LED  <--| P2.9      |
                     |   (led0)  |
                     |           |
  Kettle State In -->| P2.03     |-- Via MOSFET level shifter
                     |           |
                     |           |
 Kettle Button Out --| P2.00     |--> Via MOSFET to pull 5V line low
                     |           |
                     |           |
    Target Temp   -->| P1.06     |-- Via op-amp buffer + divider
    (ADC AIN6)       |           |
                     |           |
   Current Temp   -->| P1.07     |-- Via op-amp buffer + divider
    (ADC AIN7)       |           |
                     |           |
             GND  ---| GND       |
            3.3V  ---| VDD       |
                     +-----------+
```

### Digital Input Level Shifters (5V → 3.3V)

For digital inputs, use **N-channel MOSFET level shifters** (2N7002, ~$0.02 each). This provides high input impedance without loading the kettle's circuits.

**Note**: The MOSFET inverts the signal. The firmware is configured for `GPIO_ACTIVE_LOW` to compensate.

#### Button Input (Simple 5V digital)

```
5V Button Signal
       |
      [10K] R1
       |
       +-------+
               |
            [2N7002]        Pinout (SOT-23 top view):
            D  G  S           Pin 3 (top): Drain
            |  |  |           Pin 1 (bottom-left): Gate
            |  |  +-- GND     Pin 2 (bottom-right): Source
            |  |
            |  +-- from 10K
            |
            +---------- P2.02 (internal pull-up)
```

#### Kettle State Input (5V PWM Detection)

The kettle outputs **5V 50% PWM at ~150Hz when ON**, and **LOW when OFF**. A 10µF capacitor smooths the PWM into a stable DC level (~2.5V) for reliable MOSFET gate drive:

```
5V PWM Signal (from kettle)
       |
      [10K] R1
       |
       +-------+
       |       |
    [10µF]     |
       |    [2N7002]
      GND   D  G  S
            |  |  |
            |  |  +-- GND
            |  |
            |  +-- from 10K/10µF junction
            |
            +---------- P2.03 (internal pull-up)

RC time constant: 10K × 10µF = 100ms (smooths 150Hz PWM to ~2.5V DC)

Detection logic:
  PWM present (50%) → Cap smooths to ~2.5V DC → MOSFET ON → GPIO LOW (active)
  No PWM (LOW)      → 0V on gate → MOSFET OFF → GPIO HIGH (inactive)
```

**Parts list (inputs)**:
- Button: 1x 2N7002, 1x 10K resistor
- Kettle state: 1x 2N7002, 1x 10K resistor, 1x 10µF capacitor

### Digital Output (3.3V → 5V Open-Drain)

#### Kettle Button Output (Simulate Button Press)

To control the kettle via Zigbee, the firmware can simulate a physical button press by pulling the kettle's 5V button line low. Uses a 2N7002 MOSFET as open-drain output:

```
                    Kettle's 5V Button Line
                             |
                          [2N7002]
                          D  G  S
                          |  |  |
                          |  |  +-- GND
                          |  |
                          |  +------ P2.00 (GPIO output)
                          |
                    (to kettle button input)

GPIO HIGH (3.3V) → MOSFET ON → Pulls 5V line LOW (button pressed)
GPIO LOW         → MOSFET OFF → Line floats HIGH (button released)
```

The firmware pulses this output for 200ms when an On/Off command is received via Zigbee.

**Parts list (output)**:
- Kettle button: 1x 2N7002

### ADC Input Interface (Non-Intrusive)

**IMPORTANT**: The nRF54L15 ADC maximum input voltage is VDD (3.3V). The kettle's existing circuits output 0-5V and must NOT be loaded by adding resistive dividers directly.

**Solution: Op-amp voltage follower buffer**

Use an op-amp configured as a unity-gain buffer. This provides:
- Very high input impedance (>1MΩ) - won't affect kettle's existing circuits
- Low output impedance - can drive the voltage divider to ADC

#### Recommended Op-Amp

**MCP6001** (Microchip)
- Rail-to-rail input/output
- Single supply 1.8V to 6V
- SOT-23-5 package
- ~$0.30 each

#### Buffer Circuit (for each ADC channel)

```
Kettle Signal (0-5V)                       To nRF54L15 ADC
        |                                        |
        |     +-------+                          |
        +-----|+      |                          |
              | MCP6001|----+----[10K R1]----+---+
        +-----|-      |    |                |
        |     +-------+    |               [10K R2]
        |         |        |                |
       GND       Vdd      Vout             GND
                 (5V)   (buffered)      (0-2.5V at ADC)
```

**Op-amp power**: Connect Vdd to kettle's 5V supply, GND to common ground.

#### Target Temperature (Linear Potentiometer)

```
Kettle Temp Dial (0-5V) ---> [Buffer] ---> [10K:10K divider] ---> P1.04 (AIN0)
```
- Input: 0-5V (50°C to 100°C linear)
- After divider: 0-2.5V at ADC
- Software maps: ADC 0 = 50°C, ADC max = 100°C

#### Current Temperature (100K NTC Thermistor)

The kettle has an existing voltage divider: 5V → 10K → NTC_junction → 100K_NTC → GND

```
Kettle NTC Junction (0-4.5V) ---> [Buffer] ---> [10K:10K divider] ---> P1.05 (AIN1)
```
- At 25°C: NTC = 100K, junction voltage = 5V × 100K/(10K+100K) = 4.54V
- At 100°C: NTC ≈ 6.5K, junction voltage = 5V × 6.5K/(10K+6.5K) = 1.97V
- After divider: ~1.0V to ~2.3V at ADC (safe range)

The firmware uses the Beta parameter equation:
- Beta coefficient: 3950K (typical for 100K NTC)
- Reference resistance: 100K @ 25°C
- Accounts for the 2:1 divider after buffer

If your NTC has different parameters, update these constants in `main.c`:
```c
#define NTC_BETA    3950    /* Beta coefficient (K) */
#define NTC_R25     100000  /* Resistance at 25°C (ohms) */
```

## Building

### Prerequisites

- Python 3.8+
- J-Link debugger (for flashing)

### First-Time Setup & Build

```bash
# Clone and build (SDK downloads automatically on first run, ~4GB)
./build.sh
```

### Subsequent Builds

```bash
# Build
./build.sh

# Clean build
./build.sh nrf54l15dk clean

# Build and flash
./build.sh nrf54l15dk flash
```

### Build Outputs

- `build/firmware/zephyr/zephyr.hex` - Flash via J-Link
- `build/merged.hex` - Combined image (if using MCUboot)

Note: MCUboot/OTA is currently disabled due to SDK compatibility. See board config to re-enable.

## Zigbee2MQTT Setup

### Install External Converter

1. Copy `z2m/kitchenaid_kettle.js` to your Zigbee2MQTT data folder:
   ```bash
   cp z2m/kitchenaid_kettle.js /path/to/zigbee2mqtt/data/
   ```

2. Add to `configuration.yaml`:
   ```yaml
   external_converters:
     - kitchenaid_kettle.js
   ```

3. Restart Zigbee2MQTT

### Pairing

1. Put Zigbee2MQTT in pairing mode
2. Long-press the pairing button (3 seconds) on the kettle controller
3. The status LED will blink rapidly, then stop when paired

### Exposed Entities

| Entity | Type | Access | Description |
|--------|------|--------|-------------|
| `state` | Binary | Read | Kettle heating state (ON/OFF) |
| `current_temperature` | Numeric | Read | Current water temperature (50-100°C) |
| `target_temperature` | Numeric | Read/Write | Target temperature setpoint (50-100°C) |
| `system_mode` | Enum | Read | Heating mode (off/heat) |

### Home Assistant

After pairing with Zigbee2MQTT, the device appears in Home Assistant with:
- Binary sensor: Kettle state
- Temperature sensor: Current water temperature
- Number/Climate: Target temperature setpoint

## OTA Firmware Updates

1. Build new firmware version (increment version in `boards/nrf54l15dk_nrf54l15_cpuapp.conf`):
   ```
   CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="1.1.0"
   ```

2. Copy `build/zephyr/app_update.bin` to Zigbee2MQTT's OTA folder:
   ```bash
   cp build/zephyr/app_update.bin /path/to/zigbee2mqtt/data/otaImages/
   ```

3. Trigger OTA update from Zigbee2MQTT dashboard

## Pin Assignment Summary

| Function | Pin | Interface | Notes |
|----------|-----|-----------|-------|
| Pairing Button | P2.02 | 2N7002 MOSFET | Active low, internal pull-up |
| Status LED | P2.9 | Direct | Active high |
| Kettle State In | P2.03 | 2N7002 + 10µF | PWM detection, active low |
| Kettle Button Out | P2.00 | 2N7002 MOSFET | Open-drain output to 5V line |
| Target Temp ADC | P1.06 (AIN6) | Op-amp + divider | MCP6001 buffer, 10K:10K divider |
| Current Temp ADC | P1.07 (AIN7) | Op-amp + divider | MCP6001 buffer, 10K:10K divider |

**Note**: P1.04/P1.05 are reserved for UART20 (debug console).

## Zigbee Clusters

| Cluster | ID | Role | Description |
|---------|-----|------|-------------|
| Basic | 0x0000 | Server | Device information |
| Identify | 0x0003 | Server | Device identification |
| Groups | 0x0004 | Server | Group membership |
| On/Off | 0x0006 | Server | Kettle state (read-only) |
| Thermostat | 0x0201 | Server | Temperature setpoint |
| Temp Measurement | 0x0402 | Server | Current temperature |

## Troubleshooting

### Device won't pair
- Ensure Zigbee2MQTT is in pairing mode
- Long-press button for full 3 seconds
- Check that status LED blinks (indicates not joined)
- Try moving device closer to coordinator

### Temperature readings incorrect
- Verify voltage divider resistor values
- Check NTC thermistor wiring (should go to GND, not VDD)
- Confirm 3.3V supply for NTC circuit (not 5V)
- Update NTC_BETA and NTC_R25 if using different thermistor

### No On/Off state changes
- Verify kettle state GPIO is connected to heating element indicator
- Check GPIO pull-down configuration

### Build fails
- Ensure Python 3.8+ is installed
- Delete `.venv` and `deps` folders, then rebuild
- Check available disk space (~4GB for SDK)

## License

This project is provided as-is for educational and personal use.
