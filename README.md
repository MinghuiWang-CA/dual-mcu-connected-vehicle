# dual-mcu-connected-vehicle
This project features a high-performance robotic vehicle utilizing a dual-core architecture to balance real-time motor control with advanced wireless connectivity. The system integrates an STM32 for low-level hardware execution and an ESP32 as a high-level communication gateway.

<br>

## Key Features

* **Control Engineering**: Developed real-time PID motor control and sensor fusion algorithms on **STM32** to ensure precise movement and stability.
* **Dual-Core Connectivity**: Implemented an **ESP32 gateway** acting as a bridge for telemetry and remote commands via **WiFi and Bluetooth**.
* **Custom Communication Protocol**: Designed a specialized low-latency serial protocol (UART/SPI) for reliable data exchange between the STM32 and ESP32.
* **Real-Time OS**: Leveraged **FreeRTOS** to manage concurrent tasks such as sensor polling, motor PWM generation, and wireless data handling.
* **Interactive Control**: Integrated support for external controllers (Xbox controller) to manage direction and speed in real-time.

<br>

## System Architecture

The vehicle operates on a master-slave distributed architecture:
1.  **Communication Layer (ESP32)**: Handles the wireless stack (Bluetooth). It receives remote commands and forwards them to the controller.
2.  **Control Layer (STM32)**: Processes raw sensor data and executes the control loops (PWM) for the motors.
3.  **Power Layer**: Managed through H-bridge motor drivers (L298N/L293D) to drive the dual-track system.

<br>

## Software Stack

* **Languages**: C++ (Embedded), Python (Remote control scripts).
* **Kernel**: FreeRTOS for task scheduling and resource management.
* **Frameworks**: STM32CubeHAL, Arduino/Espressif IoT Development Framework.
* **Protocols**: UART, SPI, I2C (Sensors), MQTT/WebSockets (Wireless).

<br>

## How to Run

1.  **Hardware Setup**: Ensure the STM32 and ESP32 are wired correctly via the designated serial pins.
2.  **STM32 Firmware**: Flash the STM32 using STM32CubeIDE.
3.  **ESP32 Firmware**: Flash the ESP32 gateway code.
4.  **Control**: Connect your smartphone or computer to the "Vehicle_AP" WiFi network and launch the control interface to start driving.

<br>

## Performance Results

* **Latency**: Achieved sub-50ms response time from wireless command to motor execution.
* **Stability**: FreeRTOS task prioritization ensured that motor control loops were never interrupted by wireless network jitter.
* **Scalability**: The modular gateway design allows for easy integration of new sensors or different wireless protocols without modifying the core motor control logic.
