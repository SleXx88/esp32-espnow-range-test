# ESP32 ESP-NOW Range Test

This project evaluates the real-world communication range of Espressif's ESP-NOW protocol using ESP32-S3 modules. It demonstrates bidirectional communication, RSSI monitoring, and visual feedback via onboard RGB LEDs.

## 📡 What is ESP-NOW?

ESP-NOW is a lightweight, connectionless wireless communication protocol developed by Espressif. It allows multiple ESP32 devices to communicate directly without a Wi-Fi router, enabling ultra-low-latency and low-power data exchange. ESP-NOW supports one-to-one, one-to-many, and many-to-many topologies, making it ideal for remote control, sensor networks, and smart home applications.

## 🎯 Project Goals

- Measure the maximum reliable communication range between two ESP32-S3 boards using ESP-NOW.
- Implement bidirectional data exchange with acknowledgment.
- Monitor signal strength (RSSI) for each received packet.
- Provide visual feedback using onboard RGB LEDs:
  - **Master**: Red LED indicates data transmission.
  - **Slave**: Green LED indicates data reception.

## 🔧 Hardware Requirements

- 2× ESP32-S3 development boards (e.g., ESP32-S3-DevKitC-1)
- Onboard RGB LED connected to GPIO38 (common on many ESP32-S3 boards)
- USB cables for programming and power
- Optional: External antennas for extended range testing

## 🛠️ Software Setup

- Arduino IDE with ESP32 board support installed
- ESP32 Board Package version 2.0.0 or later
- No additional libraries required; utilizes built-in ESP-NOW and Wi-Fi libraries

## 🚀 Getting Started

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/SleXx88/esp32-espnow-range-test.git
