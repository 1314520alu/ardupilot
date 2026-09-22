# TF02PRO-Periph UART→DroneCAN Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a dedicated AP_Periph board `TF02PRO-Periph` that reads Benewake TF02-Pro over USART2 and publishes DroneCAN range_sensor at 500 kbit/s.

**Architecture:** New ChibiOS hwdef (app + bootloader) for GD32F103TBU6 treated as STM32F103xB; no shared `f103-RangeFinder` edits. Peripheral uses existing `AP_Periph` rangefinder path (`uavcan.equipment.range_sensor.Measurement`). Autopilot consumes with `RNGFND_TYPE=24`.

**Tech Stack:** ArduPilot AP_Periph, ChibiOS hwdef, DroneCAN, Benewake TF02 UART protocol (`RNGFND_TYPE=19`).

## Global Constraints

- MCU build target: `STM32F103` / `STM32F103xB` (GD32F103TBU6 pin-compatible)
- HSE: `OSCILLATOR_HZ 12000000`
- UART: PA2=USART2_TX, PA3=USART2_RX → TF02
- CAN: PA11=CAN_RX, PA12=CAN_TX @ **500000** bit/s
- No LED, no I2C/GPS/baro/compass on this PCB
- Programming: SWD first; optional DroneCAN update after BL present
- Board dir name: `TF02PRO-Periph`
- Node name: `org.ardupilot.TF02PRO`
- Board ID: `AP_HW_TF02PRO_PERIPH` = **6900** (local; Easy Aerial block unused — change if upstreaming)
- Do not commit unless the user explicitly asks
- F103 needs 12 MHz support in `libraries/AP_HAL_ChibiOS/hwdef/common/stm32f1_mcuconf.h` (PLL×6 → 72 MHz)

## File map

| File | Responsibility |
|------|----------------|
| `Tools/AP_Bootloader/board_types.txt` | Register `AP_HW_TF02PRO_PERIPH 6900` |
| `libraries/AP_HAL_ChibiOS/hwdef/common/stm32f1_mcuconf.h` | Add 12 MHz HSE clock tree for F103 |
| `libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/hwdef.dat` | App pins, clock, CAN, rangefinder enables |
| `libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/hwdef-bl.dat` | Bootloader pins/clock/CAN (no LED, no STAY pin) |
| `libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/defaults.parm` | Default `RNGFND1_TYPE=19` and range limits |

---

### Task 1: Register board ID

**Files:**
- Modify: `Tools/AP_Bootloader/board_types.txt` (near Easy Aerial reservation ~line 468)

**Interfaces:**
- Produces: symbol `AP_HW_TF02PRO_PERIPH` with numeric ID `6900` for hwdef `APJ_BOARD_ID`

- [ ] **Step 1: Add board ID entry**

Insert:

```
# IDs 6900-6909 reserved for Easy Aerial
# Local TF02PRO UART→DroneCAN adapter (GD32F103 / private)
AP_HW_TF02PRO_PERIPH                 6900
```

If `6900` is already taken in your tree, pick the next free ID in 6900–6909 and use that same number in both hwdef files.

- [ ] **Step 2: Verify symbol is unique**

Run: `grep -n TF02PRO_PERIPH Tools/AP_Bootloader/board_types.txt`  
Expected: one line with `6900` (or your chosen ID).

- [ ] **Step 3: Commit (only if user asked)**

```bash
git add Tools/AP_Bootloader/board_types.txt
git commit -m "$(cat <<'EOF'
hwdef: add AP_HW_TF02PRO_PERIPH board id

EOF
)"
```

---

### Task 2: Application hwdef + defaults

**Files:**
- Create: `libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/hwdef.dat`
- Create: `libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/defaults.parm`

**Interfaces:**
- Consumes: `AP_HW_TF02PRO_PERIPH` from Task 1
- Produces: board name `TF02PRO-Periph` buildable with `./waf configure --board TF02PRO-Periph`

- [ ] **Step 1: Write `hwdef.dat`**

Create `libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/hwdef.dat` with exact content:

```
# TF02-Pro UART → DroneCAN adapter
# MCU: GD32F103TBU6 (build as STM32F103xB)
# USART2 PA2/PA3 ↔ TF02; CAN PA11/PA12 @ 500 kbit/s; HSE 12 MHz; no LED

MCU STM32F103 STM32F103xB

FLASH_RESERVE_START_KB 25
STORAGE_FLASH_PAGE 23
define HAL_STORAGE_SIZE 800

APJ_BOARD_ID AP_HW_TF02PRO_PERIPH
env AP_PERIPH 1

OSCILLATOR_HZ 12000000
define CH_CFG_ST_FREQUENCY 1000
FLASH_SIZE_KB 128

# serial0..2 empty, serial3 = USART2 → RNGFND_PORT default 3
SERIAL_ORDER EMPTY EMPTY EMPTY USART2

PA2 USART2_TX USART2 SPEED_HIGH NODMA
PA3 USART2_RX USART2 SPEED_HIGH NODMA
define HAL_UART_NODMA

define HAL_USE_ADC TRUE
define STM32_ADC_USE_ADC1 TRUE
define HAL_DISABLE_ADC_DRIVER TRUE

define CH_CFG_ST_TIMEDELTA 0
define SERIAL_BUFFERS_SIZE 512
define PORT_INT_REQUIRED_STACK 64
define HAL_NO_TIMER_THREAD
define HAL_NO_RCOUT_THREAD
define __FPU_PRESENT 0
define DMA_RESERVE_SIZE 0

MAIN_STACK 0x200
PROCESS_STACK 0xA00

PA11 CAN_RX CAN
PA12 CAN_TX CAN
define HAL_CAN_POOL_SIZE 2500
define HAL_CAN_RX_QUEUE_SIZE 32

define HAL_UART_MIN_TX_SIZE 128
define HAL_UART_MIN_RX_SIZE 128
define HAL_UART_STACK_SIZE 256
define STORAGE_THD_WA_SIZE 300
define IO_THD_WA_SIZE 300
define HAL_DEVICE_THREAD_STACK 256

define HAL_GYROFFT_ENABLED 0

env ROMFS_UNCOMPRESSED True
define CH_DBG_ENABLE_STACK_CHECK FALSE

define CAN_APP_NODE_NAME "org.ardupilot.TF02PRO"
define HAL_CAN_BAUDRATE_DEFAULT 500000
define HAL_PERIPH_PRINTF_TO_CAN 1

# SWD flash bootloader once; do not embed BL in app ROMFS
define AP_BOOTLOADER_FLASHING_ENABLED 0
undef ROMFS
define HAL_NO_ROMFS_SUPPORT

# Rangefinder only
define AP_PERIPH_RANGEFINDER_ENABLED 1
define AP_PERIPH_RANGEFINDER_PORT_DEFAULT 3
define HAL_PERIPH_RANGEFINDER_BAUDRATE_DEFAULT 115200
define RANGEFINDER_MAX_INSTANCES 1
define AP_PERIPH_PROBE_CONTINUOUS 1

# Save flash: only Benewake TF02 serial backend
define AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED 0
define AP_RANGEFINDER_BENEWAKE_ENABLED 1
define AP_RANGEFINDER_BENEWAKE_TF02_ENABLED 1

# Disable unused periph features (explicit)
define AP_PERIPH_GPS_ENABLED 0
define AP_PERIPH_MAG_ENABLED 0
define AP_PERIPH_BARO_ENABLED 0
define AP_PERIPH_AIRSPEED_ENABLED 0
define AP_PERIPH_ADSB_ENABLED 0
define HAL_MSP_ENABLED 0

include ../include/no_bootloader_DFU.inc
```

- [ ] **Step 2: Write `defaults.parm`**

Create `libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/defaults.parm`:

```
RNGFND1_TYPE 19
RNGFND1_MIN 0.10
RNGFND1_MAX 40.0
RNGFND_BAUDRATE 115200
RNGFND_PORT 3
RNGFND_MAX_RATE 50
```

- [ ] **Step 3: Sanity-check pin script parses hwdef**

Run from repo root:

```bash
./libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/hwdef.dat --outdir /tmp/tf02pro_hwdef_check
```

Expected: exit 0; no error about missing board ID or bad pins. (If the script CLI differs on this tree, skip to Task 4 configure which exercises the same parser.)

- [ ] **Step 4: Commit (only if user asked)**

```bash
git add libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/
git commit -m "$(cat <<'EOF'
hwdef: add TF02PRO-Periph AP_Periph rangefinder board

EOF
)"
```

---

### Task 3: Bootloader hwdef

**Files:**
- Create: `libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/hwdef-bl.dat`

**Interfaces:**
- Consumes: `AP_HW_TF02PRO_PERIPH`, same crystal/CAN bitrate as app
- Produces: bootloader build via `./waf configure --board TF02PRO-Periph --bootloader`

- [ ] **Step 1: Write `hwdef-bl.dat`**

Do **not** include `f103-periph/hwdef-bl.inc` (that forces 8 MHz, LED on PA4, STAY on PB6). Use standalone content:

```
# TF02PRO-Periph bootloader — 12 MHz HSE, CAN 500 kbit/s, no LED

MCU STM32F103 STM32F103xB

FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 23
APP_START_OFFSET_KB 2

APJ_BOARD_ID AP_HW_TF02PRO_PERIPH
env AP_PERIPH 1

OSCILLATOR_HZ 12000000
define CH_CFG_ST_FREQUENCY 1000
FLASH_SIZE_KB 128

define HAL_USE_SERIAL FALSE
define CH_CFG_ST_TIMEDELTA 0
define HAL_USE_EMPTY_IO TRUE
define PORT_INT_REQUIRED_STACK 64
define __FPU_PRESENT 0
define DMA_RESERVE_SIZE 0

MAIN_STACK 0x800
PROCESS_STACK 0x800

PA11 CAN_RX CAN
PA12 CAN_TX CAN
define HAL_USE_CAN TRUE
define STM32_CAN_USE_CAN1 TRUE
define HAL_CAN_BAUDRATE_DEFAULT 500000

define HAL_BOOTLOADER_TIMEOUT 1000
```

- [ ] **Step 2: Confirm no LED / STAY pins**

Run: `grep -E 'LED|STAY_IN_BOOTLOADER|OSCILLATOR|CAN_BAUDRATE|PA2|PA11' libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/hwdef-bl.dat`  
Expected: `OSCILLATOR_HZ 12000000`, `HAL_CAN_BAUDRATE_DEFAULT 500000`, `PA11`/`PA12` present; no `LED`, no `STAY_IN_BOOTLOADER`, no USART pins.

- [ ] **Step 3: Commit (only if user asked)**

```bash
git add libraries/AP_HAL_ChibiOS/hwdef/TF02PRO-Periph/hwdef-bl.dat
git commit -m "$(cat <<'EOF'
hwdef: add TF02PRO-Periph bootloader definition

EOF
)"
```

---

### Task 4: Build bootloader and firmware

**Files:**
- Test: build outputs under `build/TF02PRO-Periph/`
- Optional copy: `Tools/bootloaders/TF02PRO-Periph_bl.bin` (via build_bootloaders.py or manual copy)

**Interfaces:**
- Consumes: Task 2 + Task 3 hwdefs
- Produces: flashable BL + app binaries

- [ ] **Step 1: Build bootloader**

```bash
./waf configure --board TF02PRO-Periph --bootloader
./waf bootloader
```

Expected: `build/TF02PRO-Periph/bin/AP_Bootloader.bin` exists; configure shows 12 MHz / board id 6900.

- [ ] **Step 2: Build AP_Periph application**

```bash
./waf configure --board TF02PRO-Periph
./waf AP_Periph
```

Expected: `build/TF02PRO-Periph/bin/AP_Periph.bin` (or `.apj`) succeeds. If flash overflow, disable more backends or shrink `HAL_CAN_POOL_SIZE` before changing pins.

- [ ] **Step 3: Copy bootloader artifact for SWD convenience (optional)**

```bash
cp build/TF02PRO-Periph/bin/AP_Bootloader.bin Tools/bootloaders/TF02PRO-Periph_bl.bin
cp build/TF02PRO-Periph/bin/AP_Bootloader.elf Tools/bootloaders/TF02PRO-Periph_bl.elf
```

- [ ] **Step 4: Commit build artifacts (only if user asked — usually skip .bin)**

Prefer committing only hwdef/source, not binaries, unless the user wants `Tools/bootloaders/` copies like `ZY_M323_F103_bl.*`.

---

### Task 5: Hardware flash and bench verification

**Files:**
- None (manual / SWD + CAN tools)

**Interfaces:**
- Consumes: binaries from Task 4
- Produces: confirmed DroneCAN node + range_sensor data

- [ ] **Step 1: SWD flash**

1. Connect SWDIO/SWCLK/GND/3V3 (or board power).
2. Flash `AP_Bootloader.bin` at flash base `0x08000000`.
3. Flash `AP_Periph.bin` at app start (after 25 KB reserve: typically `0x08000000 + 25*1024` — confirm with build map / `FLASH_RESERVE_START_KB 25`).
4. Or use GCS/DroneCAN tools that understand ArduPilot BL layout.

- [ ] **Step 2: CAN bus check**

- Bus: 500 kbit/s, termination as required.
- Open DroneCAN GUI or Mission Planner CAN inspector.
- Expected: node name contains `org.ardupilot.TF02PRO` (or DNA-assigned node with that app name).

- [ ] **Step 3: Lidar data check**

- Power TF02 on USART2 at 115200.
- Expected: `uavcan.equipment.range_sensor.Measurement` updates when distance changes.
- If no data: confirm `RNGFND1_TYPE=19`, `RNGFND_PORT=3`, wiring TX/RX not swapped, 3.3 V UART levels.

- [ ] **Step 4: Autopilot integration check**

On FC (example CAN1):

```
CAN_P1_DRIVER 1
CAN_D1_PROTOCOL 1
CAN_D1_BITRATE 500000
RNGFND1_TYPE 24
RNGFND1_ADDR 0
RNGFND1_MIN 0.1
RNGFND1_MAX 40
RNGFND1_ORIENT <mount>
```

Expected: rangefinder distance matches GUI; status Good when in range.

- [ ] **Step 5: Multi-board (if available)**

Second board: set unique `CAN_NODE` and/or `RNGFND_ADDR`; FC `RNGFND2_*` with matching ADDR. Expected: two independent distances.

---

## Spec coverage (self-review)

| Spec requirement | Task |
|------------------|------|
| New hwdef board, not edit f103-RangeFinder | 2 |
| 12 MHz HSE | 2, 3 |
| PA2/PA3 USART2, PA11/PA12 CAN | 2, 3 |
| 500 kbit/s | 2, 3 |
| No LED | 2, 3 |
| TF02 UART type + defaults | 2 (`defaults.parm`) |
| Unique APJ_BOARD_ID | 1 |
| SWD flash + verify path | 4, 5 |
| FC DroneCAN TYPE=24 | 5 |
| Multi-radar node/addr | 5 |
| Flash size confirm (64 vs 128) | Task 4 overflow handling + adjust `FLASH_SIZE_KB` if needed |

## Placeholder scan

None remaining; board ID fixed to 6900 unless collision.

## Type / name consistency

- Board directory / waf board: `TF02PRO-Periph`
- Node: `org.ardupilot.TF02PRO`
- Macro: `AP_HW_TF02PRO_PERIPH`
- Rangefinder port index: `3` everywhere (`SERIAL_ORDER`, `AP_PERIPH_RANGEFINDER_PORT_DEFAULT`, `RNGFND_PORT`)
