# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib


class Leds:
    """Controls the addressable LED strip (PWM pin 3, 30 LEDs)."""

    _LED_COUNT = 30

    def __init__(self):
        self._led = wpilib.AddressableLED(3)

        self._all_red = wpilib.AddressableLEDBuffer(self._LED_COUNT)
        self._all_green = wpilib.AddressableLEDBuffer(self._LED_COUNT)
        self._all_blue = wpilib.AddressableLEDBuffer(self._LED_COUNT)

        self._led.setLength(self._LED_COUNT)
        self._led.setColorOrder(wpilib.AddressableLED.ColorOrder.kRGB)

        for i in range(self._LED_COUNT):
            self._all_red.setRGB(i, 100, 0, 0)
            self._all_green.setRGB(i, 0, 100, 0)
            self._all_blue.setRGB(i, 0, 0, 100)

        self._led.start()

    def setAllRed(self):
        self._led.setData(self._all_red)

    def setAllGreen(self):
        self._led.setData(self._all_green)

    def setAllBlue(self):
        self._led.setData(self._all_blue)
