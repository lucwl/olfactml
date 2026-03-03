# OlfactML

Scent classification on edge hardware

## Repository Structure

| Directory               | Language   | Description                                             |
| ----------------------- | ---------- | ------------------------------------------------------- |
| [client](/client)       | TypeScript | React frontend for displaying inference results         |
| [data](/data)           | N/A        | Training/testing dataset in CSV format                  |
| [notebooks](/notebooks) | Python     | Jupyter notebooks for model training                    |
| [scripts](/scripts)     | Python     | Preprocessing and parsing scripts                       |
| [gateway](/gateway)     | Go         | BLE-WS gateway                                          |
| [sketches](/sketches)   | C/C++      | Arduino sketches for data collection and live inference |

## Hardware Configuration

BME688 Development Kit with 8x BME688 MOX sensors and a AdaFruit HUZZAH32 microcontroller.

## License

OlfactML is released under the [MIT License](LICENSE).
