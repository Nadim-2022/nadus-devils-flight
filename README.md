# Nadus Devil's Flight
Nadus Devil's Flight (NDF) is an open-source, high-performance drone flight controller.

# PIN Connection

## Pin Connections

### IMU (BNO085) — SPI
| Pico 2 GPIO | BNO085 Pin | Function    |
|-------------|------------|-------------|
| 3V3         | VIN        | Power       |
| GND         | GND        | Ground      |
| GPIO 12     | SDA        | MISO        |
| GPIO 10     | SCL        | Clock       |
| GPIO 13     | CS         | Chip select |
| GPIO 11     | DI         | MOSI        |
| GPIO 14     | INT        | Interrupt   |
| GPIO 15     | RST        | Reset       |

### Barometer (BMP581) — SPI
| Pico 2 GPIO | BMP581 Pin | Function    |
|-------------|------------|-------------|
| 3V3         | VIN        | Power       |
| GND         | GND        | Ground      |
| GPIO 12     | SDA        | MISO        |
| GPIO 10     | SCL        | Clock       |
| GPIO 16     | CSB         | Chip select |
| GPIO 11     | DI         | MOSI        |

### ELRS Receiver — UART (CRSF)
| Pico 2 GPIO | ELRS Pin | Function        |
|-------------|----------|-----------------|
| VBUS        | 5V       | Power           |
| GND         | GND      | Ground          |
| GPIO 4      | RX       | Pico TX → ELRS  |
| GPIO 5      | TX       | ELRS TX → Pico  |



# References
https://deepwiki.com/ExpressLRS/ExpressLRS/3.1-crsf-protocol-and-router
https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#single-wire-half-duplex-uart