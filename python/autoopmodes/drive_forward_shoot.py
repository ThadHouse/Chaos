# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from commands3 import Command, Scheduler
from wpilib.opmode import PeriodicOpMode, autonomous


@autonomous
class DriveForwardShoot(PeriodicOpMode):
    """Autonomous sequence: drive forward for 2 seconds, then shoot for 5 seconds."""

    def __init__(self, robot):
        super().__init__()
        self._robot = robot

        self._command = Command.sequence(
            robot.getDrive().driveForwardTime(2),
            robot.getShooter().shootTime(5),
        ).named("DriveForwardShoot")

    def disabledPeriodic(self):
        self._robot.robotPeriodic()

    def start(self):
        Scheduler.getDefault().schedule(self._command)

    def periodic(self):
        self._robot.robotPeriodic()

    def end(self):
        Scheduler.getDefault().cancel(self._command)
