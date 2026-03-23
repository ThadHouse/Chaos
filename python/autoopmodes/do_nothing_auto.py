# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from wpilib.opmode import PeriodicOpMode, autonomous


@autonomous
class DoNothingAuto(PeriodicOpMode):
    """Autonomous mode that does nothing."""

    def __init__(self, robot):
        super().__init__()
        self._robot = robot

    def disabledPeriodic(self):
        self._robot.robotPeriodic()

    def periodic(self):
        self._robot.robotPeriodic()
