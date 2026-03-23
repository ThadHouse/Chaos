# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
from commands3 import Scheduler
from commands3.button import CommandGamepad
from wpilib.framework import OpModeRobot
from wpilib.system import DataLogManager
from constants import OIConstants
from subsystems.drive_subsystem import DriveSubsystemNew
from subsystems.leds import Leds
from subsystems.shooter import Shooter


class Robot(OpModeRobot):
    """
    The methods in this class are called automatically corresponding to each
    mode. See the OpModeRobot documentation for details.
    """

    def __init__(self):
        super().__init__()

        # The robot's subsystems
        self._drive = DriveSubsystemNew()
        self._leds = Leds()
        self._shooter = Shooter(self._leds)

        self._current_reading = wpilib.AnalogInput(5)

        # The driver's controller
        self._driver_controller = CommandGamepad(OIConstants.kDriverControllerPort)

        DataLogManager.start("/home/systemcore/logs")
        DataLogManager.start()

    def getDrive(self):
        return self._drive

    def getShooter(self):
        return self._shooter

    def getLeds(self):
        return self._leds

    def getRobotCurrent(self):
        """Returns the robot's total current draw in Amps."""
        voltage = self._current_reading.getVoltage()
        current = (voltage / 3.3) * 50
        return current

    def getDriverGamepad(self):
        return self._driver_controller

    def robotPeriodic(self):
        Scheduler.getDefault().run()

    def nonePeriodic(self):
        self.robotPeriodic()


if __name__ == "__main__":
    wpilib.run(Robot)
