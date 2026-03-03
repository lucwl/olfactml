# OlfactML

Scent classification on edge hardware

## Repository Structure

| Directory               | Language   | Description                                             |
| ----------------------- | ---------- | ------------------------------------------------------- |
| [client](/client)       | TypeScript | React frontend for displaying inference results         |
| [data](/data)           | N/A        | Training/testing dataset in CSV format                  |
| [notebooks](/notebooks) | Python     | Jupyter notebooks for model training                    |
| [scripts](/scripts)     | Python     | Preprocessing and parsing scripts                       |
| [server](/server)       | Go         | BLE-WS tunnel                                           |
| [sketches](/sketches)   | C/C++      | Arduino Sketches for data collection and live inference |

## Hardware Configuration

BME688 Development Kit with 8x BME688 MOX sensors and a AdaFruit HUZZAH32 microcontroller

## Sketch Usage

1. Install the Arduino IDE
2. Follow the [this](https://github.com/boschsensortec/Bosch-BSEC2-Library/blob/master/README.md) guidance to install some necessary libraries and packages
3. Connect your board via USB
   - Select "Adafruit ESP32 Feather" in the Arduino IDE under Tools->Board

   - Select your COM Port under Tools->Port

4. Verify the code by clicking the check button in the top left corner
5. Make sure the baud rate is set to 921600 (or else, just make sure it matches the number in the code)
6. Upload the code to the board

NB: avoid touching the sensors while testing the device, as contamination on the surface may bias readings.
