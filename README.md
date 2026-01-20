# Embedded Peripheral Test Framework with UDP Gateway

## Overview
This project implements an **end-to-end peripheral test framework** for STM32,
using a **PC-based host**, a **BeagleBone Green (GBB) gateway**, and an **STM32 target**.

The system allows a PC application to send test packets over **UDP**, route them
through an embedded Linux gateway (GBB), execute peripheral loopback tests on the
STM32 (UART / I2C / SPI), and return the test results back to the PC.

The goal of the project is to demonstrate:
- Multi-layer communication (UDP → UART → Embedded peripherals)
- Packet-based protocol design
- CRC-based data integrity verification
- Clean separation between Host, Gateway, and Target roles

---

## System Roles

### PC – Host
- Initiates test requests
- Sends test packets using UDP sockets
- Receives test result packets
- Displays test outcomes to the user

### BeagleBone Green (GBB) – Gateway
- Acts as a UDP endpoint (server)
- Receives packets from the PC
- Routes packets to the STM32 over UART
- Receives test results from STM32
- Sends result packets back to the PC over UDP

> The GBB does **not** execute peripheral tests.  
> It acts strictly as a **protocol bridge / gateway**.

### STM32 – Target Under Test
- Receives test packets via UART
- Executes peripheral loopback tests:
  - UART ↔ UART
  - I2C ↔ I2C
  - SPI ↔ SPI
- Performs CRC validation
- Returns structured test results to the GBB

---

## Communication Flow

~~~
+-----------+        UDP         +----------------+       UART       +-----------+
|           |  Test Packet (UDP) |                |  Test Packet     |           |
|    PC     | -----------------> |  BeagleBone    | ---------------> |  STM32    |
|  (Host)   |                    |    Green       |                  | (Target)  |
|           | <----------------- |  (Gateway)     | <--------------- |           |
|           |  Result Packet     |                |  Test Results    |           |
+-----------+        UDP         +----------------+       UART       +-----------+
~~~

---

## Detailed Test Flow

### PC (Host)
- Builds a test packet
- Sends the packet to the GBB using UDP

### GBB (Gateway)
- Receives the UDP packet
- Forwards the packet to the STM32 over UART

### STM32 (Target)
- Parses the incoming packet
- Executes peripheral tests:
  - UART loopback test (UART ↔ UART)
  - I2C loopback test (I2C ↔ I2C)
  - SPI loopback test (SPI ↔ SPI)
- Validates data integrity using CRC
- Builds a result packet

### GBB (Gateway)
- Receives the result packet from STM32 via UART
- Sends the result packet back to the PC using UDP

### PC (Host)
- Receives test results
- Prints formatted results to the console

---

## Current Status

| Peripheral        | Status                             |
|-------------------|-------------------------------------|
| UART              | ✅ Implemented and tested           |
| I2C               | ⚠️ TX implemented (RX pending)      |
| SPI               | ⚠️ Framework implemented (debug)    |
| UDP Transport     | ✅ Implemented                      |
| Packet Protocol   | ✅ Implemented                      |
| CRC Validation    | ✅ Implemented                      |

---

## Project Structure (High Level)

~~~
.
├── Core/                     # STM32 HAL / FreeRTOS code
├── myFiles/
│   ├── packet.*              # Packet protocol implementation
│   ├── loopbackCRC.*         # Peripheral loopback + CRC logic
├── pc_tester_udp/            # PC-side UDP socket application
├── free_RTOS_Exercises.ioc
└── README.md
~~~

---

## Key Concepts Demonstrated
- Embedded-to-PC communication using UDP
- Gateway-based system architecture
- Packet-based protocol design
- CRC error detection
- FreeRTOS-based task routing
- Peripheral validation methodology

---

## Notes

This project is under active development and is designed as a **scalable framework**
rather than a one-time demo.

Planned future work includes:
- Completing SPI data path validation and stabilization
- Adding I2C RX loopback support
- Extending the packet protocol with acknowledgments, timeouts, and diagnostics
- Introducing configuration and health-monitoring packets
