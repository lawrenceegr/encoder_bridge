# Encoder Bridge — Zephyr Firmware

Zephyr RTOS firmware for a custom PCB ("encoder_bridge") using the STM32F030K6Tx
(32-pin LQFP, 32KB flash, 4KB RAM). Reads three quadrature encoders and outputs
direction pulses to a host system.

## Hardware

- **SoC**: STM32F030K6Tx (Cortex-M0, 48MHz via 8MHz HSE + PLL x6)
- **Encoders**: 3x quadrature (gpio-qdec), direction-only (steps-per-period = 1)
- **Outputs**: 6x direction GPIOs (CW/CCW per encoder), active-high 10ms pulses
- **Trigger**: PA11, bidirectional — activity indicator to host + EXTI wake input
- **Console**: USART1 on PA9/PA10, 115200 baud

### GPIO Map

| Signal  | GPIO | Dir    | Description                  |
|---------|------|--------|------------------------------|
| CHAN_1A | PA15 | Input  | Encoder 1 Phase A            |
| CHAN_1B | PB3  | Input  | Encoder 1 Phase B            |
| CHAN_2A | PB4  | Input  | Encoder 2 Phase A            |
| CHAN_2B | PB5  | Input  | Encoder 2 Phase B            |
| CHAN_3A | PB6  | Input  | Encoder 3 Phase A            |
| CHAN_3B | PB7  | Input  | Encoder 3 Phase B            |
| CCW_1   | PA4  | Output | Encoder 1 CCW pulse          |
| CW_1    | PA5  | Output | Encoder 1 CW pulse           |
| CCW_2   | PA6  | Output | Encoder 2 CCW pulse          |
| CW_2    | PA7  | Output | Encoder 2 CW pulse           |
| CCW_3   | PB0  | Output | Encoder 3 CCW pulse          |
| CW_3    | PB1  | Output | Encoder 3 CW pulse           |
| Trigger | PA11 | In/Out | Wake from sleep + activity   |
| SWDIO   | PA13 | Debug  | SWD Data                     |
| SWCLK   | PA14 | Debug  | SWD Clock                    |
| HSE     | PF0/PF1 | In  | 8MHz crystal                 |

## Project Structure

```
encoder_bridge/
├── CMakeLists.txt
├── prj.conf
├── app.overlay                  # qdec nodes, direction outputs, aliases
├── boards/others/encoder_bridge/
│   ├── board.yml                # Zephyr v2 board metadata
│   ├── board.cmake              # OpenOCD + stm32flash runners
│   ├── Kconfig.encoder_bridge   # Selects SOC_STM32F030X6
│   ├── encoder_bridge.dts       # Clock tree, USART1, GPIO enables
│   └── encoder_bridge_defconfig # Base board Kconfig
└── src/
    └── main.c                   # Firmware: encoders, trigger, sleep, UART commands
```

## Build Instructions

### Prerequisites

- Zephyr SDK 0.17.x installed
- West workspace initialized at `/home/user/zephyrproject`
- Python venv with west at `/home/user/zephyrproject/.venv`

### Build

```sh
cd /home/user/keyboards-firmware/encoder_bridge
source /home/marcus/zephyrproject/.venv/bin/activate
source /home/marcus/zephyrproject/zephyr/zephyr-env.sh

west build -b encoder_bridge . p
```

The board definition lives inside the project tree (`boards/others/encoder_bridge/`)
and is picked up automatically via `BOARD_ROOT` set in CMakeLists.txt.

### Clean Rebuild

```sh
rm -rf build
west build -b encoder_bridge .
```

### Flash (via OpenOCD)

```sh
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
  -c "program build/zephyr/zephyr.hex verify reset exit"
```

## Firmware Behavior

### Encoder Reading

Each encoder generates `INPUT_EV_REL` events via the `gpio-qdec` driver.
On an event:
- **CW** (value > 0): pulse CW_x HIGH for 10ms, CCW_x stays LOW
- **CCW** (value < 0): pulse CCW_x HIGH for 10ms, CW_x stays LOW
- Both outputs return LOW after the pulse

### Trigger Signal (PA11)

After any encoder event, PA11 is driven HIGH for 50ms as an activity
indicator to the host. PA11 is also configured as an EXTI input
(rising edge) so an external host can wake the device from sleep.

### Sleep / Wake

A 5-second inactivity timer runs from boot and resets on every encoder
event. On expiry the main thread enters `k_sleep(K_FOREVER)`. A rising
edge on PA11 from the host fires the EXTI ISR, which re-signals the
trigger and restarts the inactivity timer.

### UART Console Commands

Connect at 115200 baud on PA9/PA10. Commands are newline-terminated:

| Command     | Description                                |
|-------------|--------------------------------------------|
| `enc stat`  | Print CW and CCW counts for each encoder   |
| `enc reset` | Zero all counters                          |
| `pwr sleep` | Force immediate sleep                      |
| `pwr status`| Print uptime, last event time, idle time   |

## Memory Footprint

```
FLASH:  18,716 B / 32 KB  (57%)
RAM:     3,008 B /  4 KB  (73%)
```

## Design Decisions

The STM32F030K6 has only 32KB flash and 4KB RAM, which required several
optimizations versus the original design spec:

| Spec Feature         | Implementation                          | Reason                                      |
|----------------------|-----------------------------------------|---------------------------------------------|
| Zephyr Shell         | Lightweight UART command parser          | Shell subsystem needs ~15KB flash + 8KB RAM |
| PM subsystem         | `k_sleep(K_FOREVER)` + EXTI wake        | STM32F030 lacks LPTIM / `HAS_PM` support   |
| LOG subsystem        | `printk()` directly                     | Minimal log needs ~3KB flash overhead       |
| Input thread         | `CONFIG_INPUT_MODE_SYNCHRONOUS=y`        | Saves 1KB thread stack                      |
| Printf               | `CONFIG_CBPRINTF_NANO=y`                | Saves ~2KB flash vs full printf             |
| Clock config         | DTS only (`&clk_hse`, `&pll`, `&rcc`)   | Zephyr 4.3 derives clock Kconfig from DTS   |
| Trigger GPIO         | Defined inline in C (not `gpio-keys`)   | `gpio-keys` driver adds thread overhead     |

## References

- [gpio-qdec binding](https://docs.zephyrproject.org/latest/build/dts/api/bindings/input/gpio-qdec.html)
- [stm32f030_demo board](https://github.com/zephyrproject-rtos/zephyr/tree/main/boards/others/stm32f030_demo) (reference template)
