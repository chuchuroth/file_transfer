# Industrial Automation Closed-Loop Demonstration Test Case

## 1. Introduction

This document details a complete closed-loop industrial automation demonstration test case, showcasing the integration of various hardware components from sensing to actuation. The objective is to illustrate a practical application of IO-Link technology combined with a high-level edge controller for real-time decision-making and control.

## 2. System Overview

The demonstration system comprises the following key components:

*   **Sensing:** MAXREFDES177 (IO-Link Universal Analog Input/Output)
*   **IO-Link Master:** NUCLEO-F746ZG (Microcontroller) with TIOL221EVM (IO-Link Transceiver)
*   **Actuation:** Zimmer-LWR50L-02 (IO-Link Gripper)
*   **Edge Controller:** Raspberry Pi 5

## 3. System Architecture and Integration Logic

### 3.1. Component Roles

| Component          | Role                                  | Interface Type |
| :----------------- | :------------------------------------ | :------------- |
| **MAXREFDES177**   | IO-Link Universal Analog Input/Output (Sensor) | IO-Link        |
| **TIOL221EVM**     | IO-Link Transceiver (Part of IO-Link Master) | SPI/Pin Control |
| **NUCLEO-F746ZG**  | Microcontroller (IO-Link Master Logic) | UART/SPI/USB   |
| **Raspberry Pi 5** | High-Level Edge Controller (Processing/Control Logic) | UART/SPI/USB   |
| **Zimmer-LWR50L-02** | IO-Link Gripper (Actuator)            | IO-Link        |

### 3.2. Data Flow and Control Loop

The system operates as a closed-loop control system with the following sequence:

1.  **Sensing:** The MAXREFDES177, configured as an analog sensor (e.g., proximity, distance, or light sensor), detects a physical parameter in its environment. This data is transmitted as an IO-Link device.
2.  **Data Acquisition & Conversion:** The NUCLEO-F746ZG, acting as the IO-Link Master and utilizing the TIOL221EVM for the physical layer, receives the raw sensor data from the MAXREFDES177. The NUCLEO-F746ZG processes this IO-Link data, converting it into a standardized format suitable for higher-level processing.
3.  **Processing & Control Logic:** The Raspberry Pi 5, serving as the central edge controller, receives the processed sensor data from the NUCLEO-F746ZG (preferably via SPI for high-speed communication). It executes predefined control algorithms or logic based on the input. For instance, if the sensed value indicates the presence of an object within a certain range, the Raspberry Pi 5 will initiate an actuation command.
4.  **Actuation:** Based on the Raspberry Pi 5's decision, a command is sent back to the NUCLEO-F746ZG. The NUCLEO-F746ZG then transmits this command via IO-Link to the Zimmer-LWR50L-02 gripper, instructing it to perform a specific action (e.g., close jaws to grip an object, open jaws to release).
5.  **Feedback Loop:** The action performed by the Zimmer-LWR50L-02 (e.g., gripping an object) can be sensed by the MAXREFDES177, providing real-time feedback to the system. This feedback allows the Raspberry Pi 5 to continuously monitor the operation and make adjustments, thus closing the control loop.

### 3.3. Communication Protocols

*   **IO-Link:** Used for communication between the NUCLEO-F746ZG (Master) and the MAXREFDES177 (Device) and Zimmer-LWR50L-02 (Device). The TIOL221EVM handles the physical layer of the IO-Link communication for the NUCLEO-F746ZG.
*   **SPI:** Recommended for high-speed, reliable communication between the NUCLEO-F746ZG and the Raspberry Pi 5, facilitating efficient transfer of sensor data and control commands.

## 4. Demonstration Test Case: Object Detection and Gripping

### 4.1. Objective

To demonstrate a closed-loop industrial automation process where an object's presence is detected by a sensor, processed by an edge controller, and subsequently acted upon by a robotic gripper.

### 4.2. Setup

1.  **MAXREFDES177:** Configure as a proximity sensor. Place it in a position where it can detect the presence of an object on a conveyor belt or work surface.
2.  **Zimmer-LWR50L-02:** Mount the gripper in a fixed position above the detection area, capable of reaching and gripping objects detected by the MAXREFDES177.
3.  **TIOL221EVM & NUCLEO-F746ZG:** Connect the TIOL221EVM to the NUCLEO-F746ZG. Connect the MAXREFDES177 and Zimmer-LWR50L-02 to the IO-Link ports of the TIOL221EVM. Develop firmware for the NUCLEO-F746ZG to act as an IO-Link Master, reading data from MAXREFDES177 and sending commands to Zimmer-LWR50L-02.
4.  **Raspberry Pi 5:** Connect the Raspberry Pi 5 to the NUCLEO-F746ZG via SPI. Install necessary libraries for SPI communication and Python scripting.

### 4.3. Procedure

1.  **Initialization:**
    *   Power on all components.
    *   Ensure the NUCLEO-F746ZG (IO-Link Master) successfully establishes communication with the MAXREFDES177 and Zimmer-LWR50L-02 (IO-Link Devices).
    *   Verify SPI communication between the NUCLEO-F746ZG and Raspberry Pi 5.
    *   Initialize the Zimmer-LWR50L-02 to its open position.

2.  **Object Detection:**
    *   Place an object within the detection range of the MAXREFDES177.
    *   The MAXREFDES177 senses the object and transmits its presence (e.g., a digital HIGH signal or an analog value exceeding a threshold) to the NUCLEO-F746ZG via IO-Link.

3.  **Data Acquisition and Forwarding:**
    *   The NUCLEO-F746ZG receives the IO-Link data from the MAXREFDES177.
    *   The NUCLEO-F746ZG processes this data and forwards the object detection status to the Raspberry Pi 5 via SPI.

4.  **Decision Making (Raspberry Pi 5):**
    *   The Raspberry Pi 5 receives the object detection status.
    *   Its control logic determines that an object is present and a gripping action is required.
    *   The Raspberry Pi 5 sends a 
command (e.g., "GRIP") back to the NUCLEO-F746ZG via SPI.

5.  **Actuation (Zimmer-LWR50L-02):**
    *   The NUCLEO-F746ZG receives the "GRIP" command from the Raspberry Pi 5.
    *   It translates this into an IO-Link command and sends it to the Zimmer-LWR50L-02.
    *   The Zimmer-LWR50L-02 closes its jaws, gripping the object.

6.  **Feedback and Verification:**
    *   The MAXREFDES177 might sense the absence of the object in its original position (if the object was moved by the gripper) or a change in its analog reading, providing feedback to the NUCLEO-F746ZG.
    *   This feedback can be relayed to the Raspberry Pi 5 to confirm the gripping action and potentially trigger the next step in an automated process (e.g., move the gripped object).

### 4.4. Expected Outcome

Upon successful execution, the system should reliably detect an object and actuate the gripper to pick it up, demonstrating a functional closed-loop industrial automation process.

## 5. Software Implementation Notes

### 5.1. NUCLEO-F746ZG Firmware (C/C++)

*   **IO-Link Stack:** Implement an IO-Link Master stack to manage communication with the MAXREFDES177 and Zimmer-LWR50L-02. This will involve handling process data and potentially acyclic data for configuration.
*   **SPI Slave:** Configure the NUCLEO-F746ZG as an SPI slave to receive commands from and send data to the Raspberry Pi 5.
*   **Data Buffering:** Implement robust data buffering for both IO-Link and SPI communication to ensure data integrity and prevent loss.

### 5.2. Raspberry Pi 5 Application (Python)

*   **SPI Master:** Utilize Python libraries (e.g., `spidev`) to configure the Raspberry Pi 5 as an SPI master for communication with the NUCLEO-F746ZG.
*   **Control Logic:** Implement the object detection and gripping logic. This can be a simple `if-else` statement based on sensor readings or a more sophisticated state machine.
*   **Error Handling:** Include error handling for communication failures and unexpected sensor readings.
*   **Logging:** Implement logging to record system events, sensor data, and gripper actions for debugging and analysis.

## 6. References

[1] Texas Instruments. *TIOL221 Dual Channel IO-Link Device Evaluation Module*. [Online]. Available: https://www.ti.com/tool/TIOL221EVM
[2] STMicroelectronics. *NUCLEO-F746ZG*. [Online]. Available: https://www.st.com/en/evaluation-tools/nucleo-f746ZG.html
[3] Zimmer Group. *LWR50L-02*. [Online]. Available: https://www.zimmer-group.com/en/products/components/robotics/match-end-of-arm-ecosystem/match-grippers/lwr-hrc-02/products/lwr50l-02-00002-a
[4] Analog Devices. *MAXREFDES177*. [Online]. Available: https://www.analog.com/en/resources/reference-designs/maxrefdes177.html
[5] Pinetek Networks. *IOL HAT - IO-Link compatible Master for Raspberry Pi*. [Online]. Available: https://pinetek-networks.com/iol-hat/
