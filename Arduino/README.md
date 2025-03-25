# Arduino Implementation of Inverted Pendulum Controller

This directory contains the firmware for controlling the physical inverted pendulum system implemented on Arduino and ESP32 microcontrollers.

## Hardware Setup

- **Arduino UNO**: Main controller for high-level control (LQR algorithm implementation)
- **ESP32**: Motor control and sensor data handling
- **Rotary Encoders**: For pendulum angle and cart position sensing
- **L293D Motor Drivers**: For DC motor control
- **Custom PCB**: For integration of components

## File Structure

- `inverted_pendulum_main.ino`: Primary Arduino sketch for the UNO
- `esp32_motor_control.ino`: Motor control implementation for the ESP32
- `LQRController.h/.cpp`: Implementation of the Linear Quadratic Regulator
- `Sensors.h/.cpp`: Functions for reading and processing sensor data
- `Communication.h/.cpp`: I²C communication between Arduino and ESP32
- `Config.h`: System configuration and pin definitions
- `Utils.h/.cpp`: Utility functions and mathematical helpers

## Usage Instructions

1. Connect the Arduino UNO and ESP32 according to the wiring diagram in the report
2. Upload `esp32_motor_control.ino` to the ESP32
3. Upload `inverted_pendulum_main.ino` to the Arduino UNO
4. Open the Serial Monitor (115200 baud rate) to view debug information

## Controller Parameters

For optimal performance, we recommend the following parameters:

### PID Controller
- Kp = 1.2
- Ki = 1.63
- Kd = 0.22

### Pole Placement
- Poles at [-3, -4, -2, -3.5]
- Resulting gains: K = [12.5, 3.8, 87.2, 19.1]

### Nonlinear Controller
- Energy coefficient k2 = 1.0