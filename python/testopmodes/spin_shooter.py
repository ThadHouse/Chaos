# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from commands3 import Command, Scheduler
from wpilib.opmode import PeriodicOpMode, test_op_mode


@test_op_mode
class SpinShooter(PeriodicOpMode):
    """Test mode that continuously spins the shooter at 40 RPM."""

    def __init__(self, robot):
        super().__init__()
        self._robot = robot
        self._spin_command = robot.getShooter().getSpinCommand()

    def disabledPeriodic(self):
        self._robot.robotPeriodic()

    def start(self):
        Scheduler.getDefault().schedule(self._spin_command)

    def periodic(self):
        self._robot.robotPeriodic()

    def end(self):
        Scheduler.getDefault().cancel(self._spin_command)
