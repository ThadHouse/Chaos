# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from commands3 import Command, Scheduler
from wpilib.opmode import PeriodicOpMode, teleop


@teleop
class MainTeleop(PeriodicOpMode):
    """Teleop mode with joystick drive and shooter control."""

    def __init__(self, robot):
        super().__init__()
        self._robot = robot
        driver_gamepad = robot.getDriverGamepad()
        self._joystick_drive_command = robot.getDrive().getJoystickDriveCommand(
            driver_gamepad.getHID()
        )

    def disabledPeriodic(self):
        self._robot.robotPeriodic()

    def start(self):
        Scheduler.getDefault().schedule(self._joystick_drive_command)

        shooter = self._robot.getShooter()
        driver_gamepad = self._robot.getDriverGamepad()

        (
            driver_gamepad.rightBumper()
            .and_(driver_gamepad.leftBumper())
            .whileTrue(shooter.getSpinAndFeedCommand())
        )
        (
            driver_gamepad.rightBumper()
            .and_(driver_gamepad.leftBumper().negate())
            .whileTrue(shooter.getSpinCommand())
        )

    def periodic(self):
        self._robot.robotPeriodic()

    def end(self):
        Scheduler.getDefault().cancel(self._joystick_drive_command)
