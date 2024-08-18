from micropython import const
from machine import Pin
from time import sleep
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


def i2c_scan():
    from machine import I2C
    i2c = I2C(id=1, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
    devices = i2c.scan()
    print(f"found {len(devices)} I2C devices")
    for device in devices:
        print(f"I2C device at {device:#04x}")


def main():
    print("main() start")

    i2c_scan()

    blink = Blinkenlights()
    blink.setup()

    buttons = Buttons()
    buttons.setup()

    beeper = Beeper()
    beeper.setup()

    print("main() loop start")
    while True:
        blink.tick()
        buttons.tick()
        beeper.tick()

        if buttons.UP.PRESSED:
            print("UP")
            beeper.beep(262, 1)
            blink.max_green = blink.max_blue = 10
            blink.max_red = 255

        elif buttons.RIGHT.PRESSED:
            print("RIGHT")
            beeper.beep(277, 1)
            blink.max_red = blink.max_blue = 10
            blink.max_green = 255

        elif buttons.DOWN.PRESSED:
            print("DOWN")
            beeper.beep(294, 1)
            blink.max_red = blink.max_green = 10
            blink.max_blue = 255

        elif buttons.LEFT.PRESSED:
            print("LEFT")
            beeper.beep(311, 1)
            blink.max_red = 10
            blink.max_blue = blink.max_green = 255

        elif buttons.A.PRESSED:
            print("A")
            beeper.beep(330, 1)
            blink.max_blue = 10
            blink.max_red = blink.max_green = 255

        elif buttons.B.PRESSED:
            print("B")
            beeper.beep(349, 1)
            blink.max_green = 10
            blink.max_red = blink.max_blue = 255

        elif buttons.START.PRESSED:
            print("START")
            beeper.beep(370, 1)
            blink.max_red = blink.max_blue = blink.max_green = 255

        elif buttons.SELECT.PRESSED:
            print("SELECT")
            beeper.beep(392, 1)
            blink.max_red = blink.max_blue = blink.max_green = 128

        elif buttons.FN.PRESSED:
            print("FN")
            beeper.beep(415, 1)
            blink.max_red = blink.max_blue = blink.max_green = 10
            i2c_scan()

        sleep(0.1)


if __name__ == "__main__":
    main()

