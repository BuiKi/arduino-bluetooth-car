# 🚗 AI Bluetooth Smart Vehicle - Firmware (Arduino / C++)

This repository contains the core embedded firmware for an autonomous and remote-controlled smart robot vehicle. The system integrates real-time Bluetooth command execution with a continuous, non-blocking ultrasonic radar subsystem for proactive collision avoidance.

Developed using the **PlatformIO** ecosystem on Visual Studio Code, the codebase is fully optimized for the Microchip ATmega328P microcontroller (Arduino Uno architecture).

---

## 🏗️ System Architecture & Hardware Specs

The firmware is configured to drive an Arduino Uno board interacting with high-current motor drivers, specialized sensors, and communication modules:

- **Microcontroller:** Arduino Uno (ATmega328P @ 16MHz)
- **Motor Driver:** L298N
- **Actuator (Radar Servo):** SG90 Micro Servo (PWM-controlled positioning)
- **Range Sensor:** HC-SR04 Ultrasonic Transceiver Module
- **Wireless Comm:** HC-05 / HC-06 Bluetooth Serial Pass-through Module (Default Baud: 9600 bps)

### 📌 Pin Mapping Configurations

| Hardware Component                | Arduino Pin | Mode / Description                                              |
| :-------------------------------- | :---------: | :-------------------------------------------------------------- |
| **Motor A (Left) Speed Control**  |   `Pin 5`   | **PWM Output** (Speed modulation via `ENA`)                     |
| **Motor A Direction Control 1**   |   `Pin 6`   | **Digital Output** (`IN1_A`)                                    |
| **Motor A Direction Control 2**   |   `Pin 7`   | **Digital Output** (`IN2_A`)                                    |
| **Motor B (Right) Speed Control** |  `Pin 11`   | **PWM Output** (Speed modulation via `ENB`)                     |
| **Motor B Direction Control 1**   |  `Pin 10`   | **Digital Output** (`IN1_B`)                                    |
| **Motor B Direction Control 2**   |   `Pin 9`   | **Digital Output** (`IN2_B`)                                    |
| **HC-SR04 Ultrasonic Trigger**    |  `Pin 12`   | **Digital Output** (10µs acoustic burst excitation)             |
| **HC-SR04 Ultrasonic Echo**       |   `Pin 2`   | **Digital Input** (High-precision pulse-width capture)          |
| **SG90 Servo Control**            |   `Pin 3`   | **PWM Output** (Hardware-timed Servo position train)            |
| **Bluetooth / Serial Interface**  |   `RX/TX`   | **Hardware Serial** (Cross-linked RX/TX for wireless telemetry) |

---

## 🛠️ Advanced Firmware Features & Algorithms

The firmware stands out from standard Arduino hobbyist code due to several advanced engineering practices implemented in the `src/main.cpp` module:

### 1. Asynchronous Non-Blocking Radar (`runContinuousRadar`)

Unlike traditional implementations that lock the CPU thread using standard `delay()` loops, this firmware utilizes an **asynchronous state machine** driven by the `millis()` hardware clock. The servo sweeps back and forth between $60^\circ$ and $120^\circ$ in precise $5^\circ$ increments every 60ms. This allows the vehicle to safely process motor movements and execute driving commands simultaneously while scanning for obstacles.

### 2. Multi-Sample Digital Filtering (`getFilteredDistance`)

To mitigate acoustic noise, false reflections, and environmental signal degradation, an online digital filter is implemented. It aggregates 3 continuous data bursts and computes a filtered arithmetic mean, filtering out corrupted anomalies and out-of-bounds readings ($>200\text{ cm}$).

### 3. Dynamic Speed Scaling & Safety Override

The vehicle operates under a strict, layered safety matrix based on the distance calculated by the radar:

- **Safe Zone ($>30\text{ cm}$):** Vehicle runs at full nominal cruising speed (`motorSpeed`).
- **Warning Zone ($15\text{ cm} \le \text{Distance} \le 30\text{ cm}$):** The system triggers proactive deceleration, restricting the maximum driving velocity to $60\%$ of nominal speed to minimize kinetic impact risks.
- **Danger / Emergency Zone ($<15\text{ cm}$):** An absolute safety override flag (`safetyOverrideActive`) is flipped. The system forcefully overrides active manual inputs from Bluetooth and applies an immediate hard-braking action to preserve vehicle hardware.

---

## 📂 Project Structure

```text
car_controller/
├── .pio/               # PlatformIO compilation cache and target build binaries
├── .vscode/            # Workspace IntelliSense configurations
├── include/            # C++ Header files
├── lib/                # Third-party localized library extensions
├── src/
│   └── main.cpp        # Main application source code (State-machine and drivers)
├── .gitignore          # Repository filter excluding binary artifact folders
└── platformio.ini      # Project configuration and dependency manifest
```

````

---

## 📡 Remote Control Protocol (API Specifications)

The serial controller expects single-byte ASCII characters sent over Bluetooth at a baud rate of 9600 bps. The active command interpreter maps incoming signals according to the following control scheme:

| ASCII Command | Action / Movement | Description                                                                      |
| ------------- | ----------------- | -------------------------------------------------------------------------------- |
| **`F`**       | Move Forward      | Drives vehicle straight (Subject to dynamic speed scaling and proximity braking) |
| **`B`**       | Move Backward     | Reverses vehicle movement at nominal background speed                            |
| **`L`**       | Sharp Turn Left   | Counter-rotates left/right wheel vectors for high-mobility turning               |
| **`R`**       | Sharp Turn Right  | Clockwise wheel vector rotation for high-mobility turning                        |
| **`S`**       | Idle / Stop       | Disengages all H-Bridge driver channels instantly                                |
| **`U`**       | Speed Up          | Increases base cruising speed step-wise ($+25$ PWM points, max 255)              |
| **`D`**       | Speed Down        | Decreases base cruising speed step-wise ($-25$ PWM points, min 100)              |

---

## 🚀 Compilation & Deployment Guidelines

### Prerequisites

- Install Visual Studio Code.
- Install the PlatformIO IDE extension from the VS Code Marketplace.

### Deployment Steps

1. Open the `car_controller/` workspace directory in VS Code.
2. PlatformIO will read `platformio.ini` and automatically resolve the native Atmel AVR toolchain along with the built-in Servo dependency.
3. Connect your Arduino Uno development board via USB.
4. Click the **PlatformIO: Build** icon (Checkmark) on the bottom status bar to compile the binaries.
5. Click the **PlatformIO: Upload** icon (Right arrow) to flash the compiled binary image onto the microcontroller.
6. Connect your controller app or Python script via the Bluetooth module to begin operation!

```

```
````
