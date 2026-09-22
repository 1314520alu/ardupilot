# TF02-Pro UART → DroneCAN (AP_Periph) Design

**Date:** 2026-09-22  
**Status:** Draft for review  
**Approach:** New dedicated hwdef board (方案 1)

## Goal

Convert Benewake TF02-Pro units that currently use a small adapter PCB (UART lidar ↔ CAN) into **DroneCAN** rangefinder nodes by running **AP_Periph** on the existing MCU, without changing the TF02-Pro sensor itself.

## Hardware (existing adapter)

| Item | Value |
|------|--------|
| MCU | GD32F103TBU6 (build as STM32F103xB) |
| HSE crystal | **12 MHz** |
| TF02 UART | **PA2** = USART2_TX, **PA3** = USART2_RX |
| CAN | **PA11** = CAN_RX, **PA12** = CAN_TX |
| LED / other I/O | None |
| Programming | SWD available |
| Target CAN bitrate | **500 kbit/s** |

Data path:

```
TF02-Pro (UART 115200, Benewake)
    ↔  PA2/PA3 (USART2)
GD32F103 (AP_Periph)
    ↔  PA11/PA12 (CAN @ 500 kbit/s)
Flight controller DroneCAN bus
```

Board firmware publishes `uavcan.equipment.range_sensor.Measurement`.  
Flight controller uses `RNGFND_TYPE = 24` (DroneCAN).

## Repository changes

### New board directory

`libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/`

- `hwdef.dat` — application
- `hwdef-bl.dat` — bootloader

Do **not** modify shared `f103-RangeFinder` / `f103-periph` for this product-specific pinout and clock.

### Board identity

- `CAN_APP_NODE_NAME`: `org.ardupilot.TF02PRO`
- Allocate a **new** `APJ_BOARD_ID` in `Tools/AP_Bootloader/board_types.txt` (do not reuse `AP_HW_F103_PERIPH` long-term; shared IDs confuse DroneCAN firmware updates across different pinouts).
- Default CAN bitrate: `HAL_CAN_BAUDRATE_DEFAULT 500000`

### Clock / flash

- `OSCILLATOR_HZ 12000000`
- `FLASH_SIZE_KB 128` (adjust if part is smaller)
- Standard F103 periph layout: `FLASH_RESERVE_START_KB 25`, `STORAGE_FLASH_PAGE 23` (match proven `f103-periph` / `ZY_M323_F103` unless GD32 flash map requires change)

### Serial / rangefinder

- `SERIAL_ORDER EMPTY EMPTY EMPTY USART2`
- `AP_PERIPH_RANGEFINDER_ENABLED 1`
- `AP_PERIPH_RANGEFINDER_PORT_DEFAULT 3` (USART2 slot in SERIAL_ORDER)
- Default lidar type: Benewake TF02 (`RNGFND_TYPE = 19`)
- Default baud: 115200
- Optional: `AP_PERIPH_PROBE_CONTINUOUS 1` for slow lidar init
- No LED define
- Disable unused GPS / I2C / baro / compass backends to save flash
- `AP_BOOTLOADER_FLASHING_ENABLED 0` / no ROMFS BL embedding (SWD flash BL once, then optional CAN update)

### Bootloader

Match app: 12 MHz, CAN 500 kbit/s, PA11/PA12, same board ID.

## Parameters

### On peripheral (defaults)

| Parameter | Default | Notes |
|-----------|---------|--------|
| `RNGFND_TYPE` | 19 (BenewakeTF02) | UART protocol |
| `RNGFND_PORT` | 3 | USART2 |
| `RNGFND_BAUDRATE` | 115200 | TF02-Pro default |
| `RNGFND_ADDR` | 0 (configurable) | Becomes DroneCAN `sensor_id` |
| `RNGFND_MAX_RATE` | 50 | Cap publish rate |
| `CAN_NODE` | DNA or fixed | Unique per board on shared bus |
| CAN bitrate | 500000 | Match vehicle bus |

### Multi-sensor

Each adapter = one DroneCAN node. Differentiate with unique `CAN_NODE` (or DNA) and, if needed, unique `RNGFND_ADDR`. Autopilot instances use matching `RNGFNDx_ADDR`.

### On autopilot (one sensor example)

- `CAN_P1_DRIVER = 1`
- `CAN_D1_PROTOCOL = 1` (DroneCAN)
- Bitrate 500000 on that interface
- `RNGFND1_TYPE = 24` (DroneCAN)
- `RNGFND1_ADDR` = peripheral `RNGFND_ADDR`
- `RNGFND1_MIN` / `MAX` / `ORIENT` per mount (TF02-Pro roughly 0.1–40 m)

## Error handling

| Condition | Behavior |
|-----------|----------|
| No UART data / timeout | Do not spam stale range; FC sees NoData |
| Out of range low/high | Map to TOO_CLOSE / TOO_FAR in DroneCAN reading_type |
| Duplicate Node ID | Fix `CAN_NODE` / DNA |
| Duplicate `RNGFND_ADDR` on FC | Backend may refuse; assign unique addresses |
| Wrong crystal/bitrate | Node missing or CAN errors → verify 12 MHz and 500 kbit/s |
| GD32 quirks | If boot/flash fails, revisit flash size and page layout |

## Build & flash

1. Build bootloader for `TF02PRO-Periph`
2. Build AP_Periph firmware for `TF02PRO-Periph`
3. SWD: flash BL, then app
4. Connect CAN @ 500 kbit/s; confirm node in DroneCAN GUI / Mission Planner
5. Confirm `range_sensor` tracks real distance
6. Configure FC parameters and verify rangefinder instance

Later updates may use DroneCAN firmware update once BL is present.

## Test plan

1. SWD flash succeeds; node appears on CAN after reset  
2. UART + TF02: published range changes with target distance  
3. FC `TYPE=24` reads correct range  
4. Two boards on one bus with distinct node/address → two FC instances  
5. Disconnect TF02 → NoData; reconnect → recovers (with continuous probe if enabled)

## Out of scope

- Rewriting TF02-Pro internal lidar firmware  
- Using proprietary Benewake CAN on the flight controller for these adapters  
- Custom non–AP_Periph DroneCAN stack  
- LED / extra sensors on this PCB  

## Open items for implementation

1. Confirm GD32F103TBU6 flash size (64 vs 128 KB) on the physical part marking  
2. Final board directory / node name strings if product branding differs from `TF02PRO-Periph`

## Implementation notes

- `APJ_BOARD_ID`: `AP_HW_TF02PRO_PERIPH` = 6900 (local)
- F103 ChibiOS mcuconf previously lacked 12 MHz HSE; add `STM32_HSECLK == 12000000U` with PLL×6 → 72 MHz in `stm32f1_mcuconf.h`
