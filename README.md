# ESP32 Digital Oscilloscope

A portable, low-cost digital storage oscilloscope built using an ESP32 and an ST7789 TFT display. It is capable of visualizing waveforms in real-time and safely measuring input voltages from 0V up to 30V (with the appropriate external voltage divider). 

## Features

* **Real-Time Rendering:** High-speed sampling utilizing the ESP32's built-in I2S ADC.
* **Display Output:** Sharp, responsive waveform plotting on a 240x320 ST7789 TFT SPI display.
* **HOLD & Cursor Analysis:** Freeze the live waveform and use the dual-cursor system to zoom and manually measure voltage differences (ΔV), time differences (ΔT), and signal frequency.
* **Adjustable Timebase & Scales:** Configure timebase from 30µs to 15ms/div and voltage divisions from 0.1V to 1V.
* **Trigger Modes:** Supports AUTO, NORMAL, and SINGLE trigger modes with configurable rising/falling edges and hysteresis.
* **Self-Test Generator:** Built-in 1kHz square wave test signal available on GPIO25 for easy calibration.

## Hardware Requirements

* ESP32 Development Board
* ST7789 2.8" or 2.4" TFT Display (SPI)
* 4x Tactile Push Buttons
* Resistors for Voltage Divider (e.g., 100kΩ and 11kΩ to step down 30V to ~3.3V)
* Jumper wires and breadboard/custom PCB

## Pin Configuration

| Component / Function | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **TFT CS** | GPIO 5 | |
| **TFT DC** | GPIO 2 | |
| **TFT RST** | GPIO 4 | |
| **TFT SCK** | GPIO 18 | |
| **TFT MOSI** | GPIO 23 | |
| **Signal Input (ADC)**| GPIO 33 | **WARNING: Maximum 3.3V at this pin! Use a voltage divider for 0-30V.** |
| **Test Signal** | GPIO 25 | Outputs a 1kHz square wave for testing |
| **Button: OK / SET** | GPIO 32 | Moves through menu items or cursors |
| **Button: PLUS (+)** | GPIO 15 | Adjusts values / Moves cursor right |
| **Button: MINUS (-)**| GPIO 35 / 13 | Adjusts values / Moves cursor left |
| **Button: BACK / RUN**| GPIO 34 / 14 | Toggles between RUN and HOLD/STOP modes |

*Note: Depending on your button layout configuration in the firmware (`BUTTON_LAYOUT`), external 10k pull-down resistors may be required for GPIO34 and GPIO35.*

## Software Dependencies

Make sure to install the following libraries via the Arduino IDE Library Manager:
* [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
* [Adafruit ST7735 and ST7789 Library](https://github.com/adafruit/Adafruit-ST7735-Library)
* [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)

*It is recommended to compile this using the `arduino-esp32` core version 2.0.17.*

## Usage

1. **Connect the Hardware:** Wire the TFT display and buttons according to the pin configuration table. Ensure your voltage divider is correctly connected to GPIO33 if you are measuring voltages above 3.3V.
2. **Flash the Firmware:** Open the sketch in the Arduino IDE, select your ESP32 board, and flash the code.
3. **Test the Setup:** Use a jumper wire to connect GPIO25 to GPIO33 (temporarily bypassing the voltage divider) to view the internal 1kHz test signal.
4. **Operation:** Use the **RUN/STOP** button to freeze the waveform. Once stopped, press **SET** to cycle through the cursors (V1, V2, T1, T2) and use the **+/-** buttons to analyze specific points on the wave.
