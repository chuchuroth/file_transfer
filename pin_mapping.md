Here is the summary of the pin mapping for the closed-loop industrial system, integrating the **Raspberry Pi 5**, **NUCLEO-F746ZG**, **TIOL221EVM**, and the **Zimmer Gripper**.

### 1. High-Level Control (Raspberry Pi 5 to Nucleo)

The Pi 5 acts as the master controller, sending commands to the Nucleo via SPI based on digital sensor data (HC-SR04/PIR).

| Function | Raspberry Pi 5 Pin | NUCLEO-F746ZG Pin |
| --- | --- | --- |
| **SPI MOSI** | GPIO 10 (Pin 19) | PA7 |
| **SPI MISO** | GPIO 9 (Pin 21) | PA6 |
| **SPI SCLK** | GPIO 11 (Pin 23) | PA5 |
| **SPI CS** | GPIO 8 (Pin 24) | PA4 |
| **Common GND** | Any GND Pin | Any GND Pin |

---

### 2. IO-Link Logic (Nucleo to TIOL221EVM)

The Nucleo controls the transceiver logic to translate UART signals into 24V IO-Link pulses.

| Function | NUCLEO-F746ZG Pin | TIOL221EVM (Header J6/J7) |
| --- | --- | --- |
| **TX (Data In)** | PA0 (UART4_TX) | **J6, Pin 1** |
| **RX (Data Out)** | PA1 (UART4_RX) | **J6, Pin 2** |
| **Enable (EN)** | PD2 (GPIO) | **J6, Pin 3** |
| **Wake-Up** | PG2 (GPIO) | **J6, Pin 4** |
| **Logic GND** | GND | **J7, Pin 3** |

---

### 3. Field Connection (TIOL221EVM to Zimmer Gripper)

This connection handles both the communication (C/Q) and the dual-power requirement for the Class B robot module.

| Zimmer Pin | Wire Color | Function | Connection Point |
| --- | --- | --- | --- |
| **Pin 1** | Brown | **L+** (Logic 24V) | TIOL221EVM **J2, Pin 1** |
| **Pin 2** | White | **P24** (Actuator 24V) | **+24V** Industrial PSU |
| **Pin 3** | Blue | **L-** (Logic GND) | TIOL221EVM **J3, Pin 1** |
| **Pin 4** | Black | **C/Q** (Data) | TIOL221EVM **J2, Pin 2** |
| **Pin 5** | Gray | **N24** (Actuator GND) | **GND** Industrial PSU |

---

### 4. Direct Sensors (Sensing to Raspberry Pi 5)

The digital sensors are wired directly to the Pi's GPIO for real-time processing.

* **HC-SR04 (Ultrasonic):** Trig to **GPIO 23**; Echo to **GPIO 24** (via 5V-to-3.3V divider).
* **HC-SR501 (PIR):** Out to **GPIO 25**.
* **Power:** Both sensors use the Pi’s **5V** and **GND** rails.


