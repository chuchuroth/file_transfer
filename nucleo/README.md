# NUCLEO-F746ZG Firmware — Build & Flash Instructions

## Overview

The firmware runs on the **NUCLEO-F746ZG** (STM32F746ZG MCU).  It acts as:
- **SPI1 slave** — receives 1-byte commands from the Raspberry Pi 5.
- **IO-Link master** — drives the Zimmer LWR50L-02 gripper via the TIOL221EVM
  transceiver over UART4.

---

## Required Tools

| Tool | Version | Download |
|---|---|---|
| STM32CubeIDE | ≥ 1.15 | https://www.st.com/en/development-tools/stm32cubeide.html |
| STM32CubeF7 HAL | ≥ 1.17 | bundled with CubeIDE |
| ST-Link/V3 driver | latest | bundled with CubeIDE |

A GNU `arm-none-eabi` standalone toolchain also works; see the *Makefile build*
section at the bottom.

---

## Step 1 — Create a New CubeIDE Project

1. **File → New → STM32 Project**.
2. Search for `NUCLEO-F746ZG` and select it.  Click **Next**.
3. Name the project (e.g. `industrial_automation_nucleo`).
4. Select **C** language, STM32Cube HAL. Click **Finish**.
5. Accept CubeMX perspective.

---

## Step 2 — Configure Peripherals in CubeMX

### SPI1 — Slave (RPi5 ↔ Nucleo)

| Setting | Value |
|---|---|
| Mode | Full-Duplex Slave |
| Hardware NSS Signal | Input Signal |
| Data Size | 8 Bits |
| First Bit | MSB First |
| CPOL | Low |
| CPHA | 1 Edge |

Pins auto-assigned: **PA4** (NSS), **PA5** (SCK), **PA6** (MISO), **PA7** (MOSI).

Enable **SPI1 global interrupt** in the NVIC tab.

### UART4 — IO-Link COM2

| Setting | Value |
|---|---|
| Mode | Asynchronous |
| Baud Rate | 38400 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |

Pins auto-assigned: **PA0** (TX), **PA1** (RX).

### GPIO Outputs

| Pin | Label | Default State |
|---|---|---|
| PD2 | `IOLINK_EN` | Low |
| PG2 | `IOLINK_WAKE` | Low |

In the *Pinout* view, right-click PD2 → GPIO\_Output, then enter `IOLINK_EN` in
the *User Label* field.  Repeat for PG2 / `IOLINK_WAKE`.

### System Core

- **SYS → Timebase Source**: choose any timer (e.g. TIM1) to free up SysTick
  for HAL.
- Enable **DWT** (Data Watchpoint and Trace) for µs-accurate wake-up timing:
  this is done in `main.c` via `dwt_enable()`.

---

## Step 3 — Generate Code

Click **Project → Generate Code**.  CubeIDE creates the project skeleton with
all HAL init functions filled in automatically.

---

## Step 4 — Add Application Source Files

Copy the provided files into the generated project:

```
nucleo/Core/Inc/main.h    → <project>/Core/Inc/main.h     (replace)
nucleo/Core/Inc/iolink.h  → <project>/Core/Inc/iolink.h   (new)
nucleo/Core/Src/main.c    → <project>/Core/Src/main.c     (replace)
nucleo/Core/Src/iolink.c  → <project>/Core/Src/iolink.c   (new)
```

> **Note:** The `main.c` provided here contains `MX_*_Init()` stub functions
> that are already filled out. When you replace the CubeMX-generated `main.c`,
> the full HAL init bodies you generated in Step 3 should be retained.
> The easiest workflow is:
> 1. Let CubeMX generate `main.c`.
> 2. Open it and locate `/* USER CODE BEGIN 2 */`.
> 3. Paste the application loop and callback from the provided `main.c` into
>    the appropriate USER CODE sections.

---

## Step 5 — Build

In CubeIDE: **Project → Build All** (or press **Ctrl+B**).

Expected output in the *Console*:
```
arm-none-eabi-size industrial_automation_nucleo.elf
   text    data     bss     dec     hex filename
  12xxx     xxx     xxx   xxxxx    xxxx industrial_automation_nucleo.elf
Finished building: industrial_automation_nucleo.elf
```

---

## Step 6 — Flash

Connect the NUCLEO board via USB.  In CubeIDE:

**Run → Run** (or press **F11**).

The ST-Link on the NUCLEO board programs the MCU automatically.

---

## Optional: Makefile Build (arm-none-eabi standalone)

If you prefer a command-line build without CubeIDE, install the STM32CubeF7
firmware package and the ARM toolchain:

```bash
# Install toolchain (Debian/Ubuntu/RPiOS)
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi

# Clone STM32CubeF7 HAL (or use the copy from your CubeIDE installation)
git clone https://github.com/STMicroelectronics/STM32CubeF7.git ~/STM32CubeF7
```

Then adapt the standard STM32 Makefile template (available at
https://github.com/STMicroelectronics/STM32CubeF7/tree/master/Projects) to
include `Core/Src/iolink.c` alongside the generated sources.

---

## Pin Wiring Summary

| NUCLEO Pin | Signal | Connect To |
|---|---|---|
| PA4 | SPI1_NSS | RPi5 GPIO 8 (Pin 24) |
| PA5 | SPI1_SCK | RPi5 GPIO 11 (Pin 23) |
| PA6 | SPI1_MISO | RPi5 GPIO 9 (Pin 21) |
| PA7 | SPI1_MOSI | RPi5 GPIO 10 (Pin 19) |
| GND | Common GND | RPi5 GND |
| PA0 | UART4_TX | TIOL221EVM J6 Pin 1 |
| PA1 | UART4_RX | TIOL221EVM J6 Pin 2 |
| PD2 | IOLINK_EN | TIOL221EVM J6 Pin 3 |
| PG2 | IOLINK_WAKE | TIOL221EVM J6 Pin 4 |
| GND | Logic GND | TIOL221EVM J7 Pin 3 |
