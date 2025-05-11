# ESP32 TWAI ISO-TP Library

**Version: 0.1.1** (or your current version)
**Author: Kostovite (minhhieudo2545@gmail.com)**
**License: MIT License** (Assuming you chose MIT, adjust if different)

## Overview

This Arduino library provides a foundational implementation of the ISO 15765-2 (ISO-TP) transport protocol layer, designed to run on ESP32 microcontrollers using their built-in TWAI (CAN) peripheral. It enables the sending and receiving of multi-frame CAN messages, which is essential for many OBD-II diagnostic services (like reading VIN, DTCs) and other CAN-based communication protocols that exceed the standard 8-byte CAN frame payload.

This library handles the segmentation, reassembly, and basic flow control aspects of ISO-TP.

## Features

*   Supports sending and receiving ISO-TP messages (Single Frame, First Frame, Consecutive Frame, Flow Control).
*   Designed for use with the ESP32's native TWAI (CAN) controller.
*   Configurable Flow Control parameters (Block Size, STmin) for frames sent by the ESP32.
*   Handles basic ISO-TP timing parameters (N_Bs, N_Cr, and a general activity timeout).
*   Allows custom CAN frame padding byte.
*   Provides a simple API for integrating into OBD-II readers or other ISO-TP applications.

## Prerequisites

*   Arduino IDE (1.8.13 or newer recommended).
*   **ESP32 Board Support Package installed** in the Arduino IDE.
*   An ESP32 development board (e.g., ESP32-WROOM-DA, ESP32 Dev Module).
*   A CAN Transceiver module (e.g., Waveshare SN65HVD230, MCP2551 based boards).

## Hardware Setup (Example with Waveshare SN65HVD230)

This library was tested using an ESP32 and a Waveshare SN65HVD230 CAN transceiver module.

**Connections (ESP32 powered via USB or onboard sources):**

1.  **ESP32 to CAN Transceiver (SN65HVD230):**
    *   `ESP32 GPIO for CAN_TX` (e.g., `GPIO22`) -> `CAN TX`
    *   `ESP32 GPIO for CAN_RX` (e.g., `GPIO21`) -> `CAN RX`
    *   `ESP32 3V3` -> `Transceiver 3.3V VCC`
    *   `ESP32 GND` -> `Transceiver GND`

2.  **CAN Transceiver to OBD-II Port (or other CAN bus):**
    *   `Transceiver CANH` -> `OBD-II Pin 6` (CAN High)
    *   `Transceiver CANL` -> `OBD-II Pin 14` (CAN Low)*