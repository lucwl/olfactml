# sketches

Arduino sketches for data collection and live inference.

## Arduino IDE Setup

1. Install the Arduino IDE
2. Follow the [this](https://github.com/boschsensortec/Bosch-BSEC2-Library/blob/master/README.md) guidance to install some necessary libraries and packages
3. Connect your board via USB
   - Select "Adafruit ESP32 Feather" in the Arduino IDE under Tools->Board

   - Select your COM Port under Tools->Port

4. Verify the code by clicking the check button in the top left corner
5. Make sure the baud rate is set to 921600 (or else, just make sure it matches the number in the code)
6. Upload the code to the board

NB: avoid touching the sensors while testing the device, as contamination on the surface may bias readings.

## Main Components

The primary sketches used:

1. `inference.ino`: real-time inference and configuration
2. `logging.ino`: data collection plan execution and logging to SD card
