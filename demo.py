from micropython import const
from machine import Pin
from time import sleep, sleep_ms
from random import randint


# derived from dc32_badge_schematic.pdf
# connect to GND when pressed
SW_START = const(22)
SW_SELECT = const(23)
SW_A = const(21)
SW_B = const(20)
SW_UP = const(18)
SW_DOWN = const(17)
SW_LEFT = const(19)
SW_RIGHT = const(16)
SW_FN = const(24)

RGB_DATA = const(4)
NUM_LEDS = const(10)
SPEAKER_OUT = const(25)
SYS_POWER_CONTROL = const(11)
TOUCH_INT = const(1)

I2C_SDA = const(2)
I2C_SCL = const(3)

# I2C devices
# LIS3DHTR accelerometer
# PCF8563T/5,518 Real-Time Clock
# NS2009 Resistive Touch Controller

# Shitty Add-on port GPIO
USER1 = const(28)
USER2 = const(29)

# IR transceiver:  ZHX1010MV115TH
IR_DR = const(7)
IR_TX = const(26)
IR_RX = const(27)

# display SPI: CH240QV23A-T
DISPLAY_RS = const(5)
DISPLAY_DI = const(6)
DISPLAY_SCK = const(8)
DISPLAY_CS = const(9)
DISPLAY_BL = const(10)
# TODO fix rotation with proper MADCTL
DISPLAY_WIDTH = const(240)
DISPLAY_HEIGHT = const(320)

# micro SD
SPI_DO = const(12)
SD_CS = const(13)
SPI_CK = const(14)
SPI_DI = const(15)


class Actor:
    def setup(self):
        pass

    def tick(self):
        pass


class Blinkenlights(Actor):
    def setup(self):
        self.min_red = 0
        self.max_red = 255

        self.min_green = 0
        self.max_green = 255

        self.min_blue = 0
        self.max_blue = 255

        from neopixel import NeoPixel
        self.np = NeoPixel(Pin(RGB_DATA, Pin.OUT), NUM_LEDS)
        for i in range(NUM_LEDS):
            self.np[i] = (0, 0, 0)
        self.np.write()

    def tick(self):
        for i in range(NUM_LEDS-1, 0, -1):
            self.np[i] = self.np[i-1]
        self.np[0] = (
            randint(self.min_red, self.max_red),
            randint(self.min_green, self.max_green),
            randint(self.min_blue, self.max_blue),
        )
        self.np.write()


class Button(Actor):
    def __init__(self, gpio):
        self.gpio = gpio

    def setup(self):
        self.pin = Pin(self.gpio, Pin.IN, Pin.PULL_UP)
        self.state = 1
        self.prev_state = 1
        self.DOWN = False
        self.UP = True
        self.PRESSED = False
        self.RELEASED = False

    def tick(self):
        self.prev_state = self.state
        self.state = self.pin.value()
        self.DOWN = self.state == 0
        self.UP = self.state == 1
        self.PRESSED = self.state == 0 and self.prev_state == 1
        self.RELEASED = self.state == 1 and self.prev_state == 0


class Buttons(Actor):
    def setup(self):
        self.START = Button(SW_START)
        self.SELECT = Button(SW_SELECT)
        self.A = Button(SW_A)
        self.B = Button(SW_B)
        self.FN = Button(SW_FN)
        self.UP = Button(SW_UP)
        self.DOWN = Button(SW_DOWN)
        self.LEFT = Button(SW_LEFT)
        self.RIGHT = Button(SW_RIGHT)

        self._buttons = [
            x for x in self.__dict__.values() if isinstance(x, Button)
        ]
        for b in self._buttons:
            b.setup()

    def tick(self):
        for b in self._buttons:
            b.tick()


class Beeper(Actor):
    def setup(self):
        from machine import PWM
        self.pwm = PWM(Pin(SPEAKER_OUT), freq=2000, duty_u16=0)
        self.freq = self._freq = 0
        self.duration = -1

    def tick(self):
        if self._freq != self.freq:
            self.pwm.freq(self.freq)
            self.pwm.duty_u16(0x8000)
            self._freq = self.freq

        elif self.duration > 0:
            self.duration -= 1

        elif self.duration == 0:
            self.pwm.duty_u16(0)
            self._freq = self.freq = 0
            self.duration = -1

    def beep(self, freq=100, duration=5):
        self.freq = freq
        self.duration = duration


class Display(Actor):
    def setup(self):
        # TODO somehow the display is not wired correctly for SPI0 :(
        #from machine import SPI
        #self.spi = SPI(0, baudrate=10_000_000, bits=8, sck=Pin(DISPLAY_SCK), mosi=Pin(DISPLAY_DI))
        from machine import SoftSPI
        self.spi = SoftSPI(baudrate=40_000_000, sck=Pin(DISPLAY_SCK), mosi=Pin(DISPLAY_DI), miso=26)
        self.cs = Pin(DISPLAY_CS, Pin.OUT, value=1)
        self.bl = Pin(DISPLAY_BL, Pin.OUT, value=0)
        self.dc = Pin(DISPLAY_RS, Pin.OUT, value=0)

        # SWRESET
        self._send_cmd(0x01)
        sleep_ms(150)

        # SLPOUT
        self._send_cmd(0x11)
        sleep_ms(500)

        # COLMOD (16bit/pixel)
        self._send_cmd(0x3A, b"\x55")
        sleep_ms(10)

        # MADCTL
        self._send_cmd(0x36, b"\x00")

        # NORON
        self._send_cmd(0x13)
        sleep_ms(10)

        # DISPON
        self._send_cmd(0x29)
        sleep_ms(500)

        # backlight on
        self.bl(1)

        print("disp on")

        self.clear()
        print("disp cleared")

    @micropython.native
    def clear(self):
        c = 8
        buff = b"\x00\x00" * DISPLAY_WIDTH * c
        for y in range(0, DISPLAY_HEIGHT, c):
            self._send_pixels(0, y, DISPLAY_WIDTH - 1, y + c - 1, buff)

    @micropython.native
    def clear_rect(self, x0, y0, x1, y1, r=0, g=0, b=0):
        w = x1 - x0
        h = y1 - y0
        u = (r & 0xF8) | ((g & 0xE0) >> 5)
        v = ((g & 0x1C) << 3) | (b >> 3)
        buff = bytes((u, v)) * w * h
        self._send_pixels(x0, y0, x1 - 1, y1 - 1, buff)

    def _send_cmd(self, cmd, data=b""):
        self.cs(0)
        self.dc(0)
        self.spi.write(bytes((cmd,)))
        if len(data) > 0:
            self.dc(1)
            self.spi.write(data)
        self.cs(1)

    @micropython.native
    def _send_pixels(self, x0, y0, x1, y1, buff):
        buff1 = bytearray(1)
        buff4 = bytearray(4)
        dc = self.dc
        cs = self.cs
        spi = self.spi

        cs(0)

        # CASET
        dc(0)
        buff1[0] = 0x2A
        spi.write(buff1)

        dc(1)
        buff4[0] = x0 >> 8
        buff4[1] = x0 & 0xFF
        buff4[2] = x1 >> 8
        buff4[3] = x1 & 0xFF
        spi.write(buff4)

        # RASET
        dc(0)
        buff1[0] = 0x2B
        spi.write(buff1)

        dc(1)
        buff4[0] = y0 >> 8
        buff4[1] = y0 & 0xFF
        buff4[2] = y1 >> 8
        buff4[3] = y1 & 0xFF
        spi.write(buff4)

        # RAMWR
        dc(0)
        buff1[0] = 0x2C
        spi.write(buff1)

        dc(1)
        spi.write(buff)

        cs(1)

    def tick(self):
        pass


class Touchscreen(Actor):
    def __init__(self, i2c):
        self.i2c = i2c

    def setup(self):
        self.x = self.y = self.z = 0
        self.addr = 0x48
        self.z_thr = 30

    def _read(self, cmd):
        buff = self.i2c.readfrom_mem(self.addr, cmd, 2)
        v = (buff[0] << 4) | (buff[1] >> 4)
        return v

    def tick(self):
        self.z = self._read(0xe0)
        if self.z > self.z_thr:
            self.x = self._read(0xc0)
            self.y = self._read(0xd0)
            print(f"touch x={self.x} y={self.y} z={self.z}")
        else:
            self.x = self.y = -1


def i2c_scan(i2c):
    devices = i2c.scan()
    print(f"found {len(devices)} I2C devices")
    for device in devices:
        print(f"I2C device at {device:#04x}")


def main():
    print("main() start")

    from machine import I2C
    i2c = I2C(id=1, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))

    i2c_scan(i2c)

    blink = Blinkenlights()
    blink.setup()

    buttons = Buttons()
    buttons.setup()

    beeper = Beeper()
    beeper.setup()

    display = Display()
    display.setup()

    touchscreen = Touchscreen(i2c)
    touchscreen.setup()

    print("main() loop start")
    while True:
        blink.tick()
        buttons.tick()
        beeper.tick()
        display.tick()
        touchscreen.tick()

        if buttons.UP.PRESSED:
            print("UP")
            beeper.beep(262, 1)
            blink.max_green = blink.max_blue = 10
            blink.max_red = 255
            display.clear_rect(8, 8, 8+32, 8+32, r=255)

        elif buttons.RIGHT.PRESSED:
            print("RIGHT")
            beeper.beep(277, 1)
            blink.max_red = blink.max_blue = 10
            blink.max_green = 255
            display.clear_rect(8, 8, 8+32, 8+32, g=255)

        elif buttons.DOWN.PRESSED:
            print("DOWN")
            beeper.beep(294, 1)
            blink.max_red = blink.max_green = 10
            blink.max_blue = 255
            display.clear_rect(8, 8, 8+32, 8+32, b=255)

        elif buttons.LEFT.PRESSED:
            print("LEFT")
            beeper.beep(311, 1)
            blink.max_red = 10
            blink.max_blue = blink.max_green = 255
            display.clear_rect(8, 8, 8+32, 8+32, b=255, g=255)

        elif buttons.A.PRESSED:
            print("A")
            beeper.beep(330, 1)
            blink.max_blue = 10
            blink.max_red = blink.max_green = 255
            display.clear_rect(8, 8, 8+32, 8+32, r=255, g=255)

        elif buttons.B.PRESSED:
            print("B")
            beeper.beep(349, 1)
            blink.max_green = 10
            blink.max_red = blink.max_blue = 255
            display.clear_rect(8, 8, 8+32, 8+32, r=255, b=255)

        elif buttons.START.PRESSED:
            print("START")
            beeper.beep(370, 1)
            blink.max_red = blink.max_blue = blink.max_green = 255
            display.clear_rect(8, 8, 8+32, 8+32, r=255, g=255, b=255)

        elif buttons.SELECT.PRESSED:
            print("SELECT")
            beeper.beep(392, 1)
            blink.max_red = blink.max_blue = blink.max_green = 128
            display.clear_rect(8, 8, 8+32, 8+32, r=128, g=128, b=128)

        elif buttons.FN.PRESSED:
            print("FN")
            beeper.beep(415, 1)
            blink.max_red = blink.max_blue = blink.max_green = 10
            i2c_scan(i2c)

        elif touchscreen.x != -1:
            # TODO is this calibration universal?
            tx = (touchscreen.x - 350) / 3500

            r = randint(0, 255)
            g = randint(0, 255)
            b = randint(0, 255)
            x = int((1-tx) * DISPLAY_HEIGHT)
            x = max(0, min(DISPLAY_HEIGHT - 32, x))
            display.clear_rect(128, x, 128+32, x+32, r=r, g=g, b=b)

            f = int(touchscreen.x / 0x1000 * 700 + 100)
            beeper.beep(f, 1)

        sleep(0.1)


if __name__ == "__main__":
    main()

