# Defcon 32 Badge - MicroPython PoC

Some example code to drive the hardware of the Defcon 32 badge with MicroPython.


## Things (Not) Implemented

- [x] Buttons
- [x] LEDs
- [x] Speaker
- [x] Display
- [x] Touchscreen
- [ ] RTC
- [x] Accelerometer
- [x] Voltage sensing
- [ ] IrDA
- [ ] SD card


## Installation Instructions

* Download MicroPython firmware from: https://micropython.org/download/RPI_PICO2/
* Hold BOOT button on badge (closest button to IR interface)
* Connect USB cable
* Removable storage device should appear
* Copy MicroPython u2f file to storage device
* Badge should re-enumerate as serial device
* Install & use mpremote: https://docs.micropython.org/en/latest/reference/mpremote.html
* Run program once: `mpremote a0 run demo.py`
* Install program on badge: `mpremote a0 fs cp demo.py :main.py`

## Usage Instructions

* Press POWER button to power on
* Press RESET button to power off
* Press other buttons for beeps and colors
* Connect to serial interface to receive output
* Send ctrl-c via serial to drop into Python REPL

