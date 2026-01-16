# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

KitchenAid 5KEK1522 Zigbee Kettle Controller - Nordic nRF54L15-based Zigbee Router firmware that enables smart home connectivity for the KitchenAid kettle. The device monitors kettle state, temperature, and allows remote control via Zigbee2MQTT.

## Build Commands

```bash
# First-time setup (downloads ~4GB SDK, creates venv)
./build.sh

# Regular build
./build.sh

# Clean build
./build.sh clean

# Full rebuild (deletes build directory)
./build.sh pristine

# Build and flash via J-Link
./build.sh flash

# Custom board target
./build.sh <board_name>
```

Build output: `build/firmware/zephyr/zephyr.hex`

## Architecture

### Firmware Structure
- **`firmware/src/main.c`** - Complete application: Zigbee clusters, state machine, ADC sampling, GPIO handling
- **`firmware/include/zb_kettle.h`** - Zigbee device macros, cluster definitions, endpoint descriptors
- **`firmware/boards/*.overlay`** - Device tree: pin assignments, ADC channels, timer allocation
- **`firmware/boards/*.conf`** - Board-specific Kconfig: crystal, crypto (CRACEN), RRAM settings

### Zigbee Clusters (Endpoint 1)
| Cluster | ID | Purpose |
|---------|-----|---------|
| Basic | 0x0000 | Device info (KitchenAid, 5KEK1522-ZB) |
| On/Off | 0x0006 | Kettle heating state |
| Thermostat | 0x0201 | Target temperature setpoint |
| Temp Measurement | 0x0402 | Current water temperature |

### State Machine
```
KETTLE_STATE_OFF → TURNING_ON → KETTLE_STATE_ON → TURNING_OFF → OFF
```
- 5-second timeout for declined commands (e.g., no water)
- State tracked from kettle's GPIO output (source of truth)

### Hardware Interface
- **Kettle State In (P2.03)**: GPIO input detecting kettle heating via PWM signal
- **Kettle Button Out (P2.00)**: GPIO output simulating button press via MOSFET
- **ADC Channels**: Op-amp buffered inputs for target temp (potentiometer) and current temp (NTC thermistor)
- **Status LED (P2.09)**: Blinks when not joined, off when joined

### Temperature Conversion
- Target temp: Linear mapping from potentiometer ADC (50-100°C range)
- Current temp: NTC thermistor using Beta equation (Beta=3950K, R25=100K)
- All Zigbee temperatures in 0.01°C units (multiply by 100)

### Z2M Integration
- **`z2m/kitchenaid_kettle.js`** - External converter for Zigbee2MQTT
- Install to: `data/external_converters/kitchenaid_kettle.js`
- Exposes: state (controllable), current_temperature, target_temperature, system_mode

## Key Implementation Notes

1. **GPIO Polling Required**: Interrupts not functional on nRF54L15; main loop polls at 50ms intervals
2. **Non-Intrusive ADC**: Op-amp buffers prevent loading kettle's voltage dividers
3. **Persistent Settings**: Target temperature saved to NVS via settings subsystem
4. **Router Mode**: Device never sleeps, always forwards Zigbee messages
5. **MCUboot Disabled**: OTA updates disabled by default due to SDK compatibility; enable via `sysbuild_mcuboot.conf`

## SDK & Toolchain

- nRF Connect SDK v2.9.2 via `ncs-zigbee` v1.2.1 add-on
- Dependencies managed by West manifest (`west.yml`)
- SDK downloaded to `deps/` directory (~4GB)
- Python venv in `.venv/` for build tools
