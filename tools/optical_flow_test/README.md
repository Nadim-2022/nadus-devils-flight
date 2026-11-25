# MTF-01P bench test

Reads the MTF-01P on the PC through a USB to UART adapter, before it goes on the Pico.
Same Micolink decoding as `src/sensors/Mtf01p.cpp`.

## Wiring

| MTF-01P | USB to UART adapter |
|---|---|
| 5V | 5V (VCC on the 5V setting) |
| GND | GND |
| Tx | RX |
| Rx | TX (only needed for MicoAssistant) |

Adapter logic level on 3.3 V. The sensor runs on 5 V but its UART is 3.3 V.

Set the sensor to **Micolink** in MicoAssistant first, it comes set to AUTO.

## Run

```
pip install pyserial
python3 mtf01p_test.py                    # live print
python3 mtf01p_test.py --raw              # hex dump of what the sensor sends
python3 mtf01p_test.py --port /dev/ttyUSB1 --baud 115200
```

If the port won't open: `sudo usermod -aG dialout $USER`, then log out and back in.

## Guided tests

Each one says what to do, collects 3 s per step and prints one summary.

```
python3 mtf01p_test.py --test height      # 30 cm, 1 m, 2 m against a tape measure
python3 mtf01p_test.py --test covered     # good vs covered: which status values mean good
python3 mtf01p_test.py --test floor       # flow quality at 1 m over different floors
python3 mtf01p_test.py --test direction   # slide forward then right at 50 cm, prints the main.cpp settings
```

Send the four summaries back so the values in main.cpp can be set.
