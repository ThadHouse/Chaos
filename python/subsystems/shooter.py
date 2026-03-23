# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
from commands3 import Mechanism, Command, Scheduler
from subsystems.leds import Leds
from constants import ShooterConstants


class Shooter(Mechanism):
    """Shooter mechanism with velocity control and ball feeding."""

    def __init__(self, leds: Leds):
        super().__init__()
        self._leds = leds
        self._last_speed = 0.0

        self._shooter_motor = wpilib.ExpansionHubMotor(1, ShooterConstants.kShooterMotorPort)
        self._left_feeder_servo = wpilib.ExpansionHubServo(1, 0)
        self._right_feeder_servo = wpilib.ExpansionHubServo(1, 2)

        self._shooter_motor.setReversed(True)
        self._shooter_motor.setDistancePerCount(ShooterConstants.kEncoderDistancePerPulse)
        self._shooter_motor.setEnabled(True)

        self._left_feeder_servo.setContinousRotationMode(True)
        self._right_feeder_servo.setContinousRotationMode(True)

        self._left_feeder_servo.setReversed(True)

        self._left_feeder_servo.setEnabled(True)
        self._right_feeder_servo.setEnabled(True)

        pid_constants = self._shooter_motor.getVelocityPidConstants()
        pid_constants.setPID(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
        pid_constants.setFF(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA)

        self.setDefaultCommand(
            self.runRepeatedly(lambda: (self.setSpeed(0), self.setFeed(False)))
            .withPriority(Command.LOWEST_PRIORITY)
            .named("Default Shooter")
        )

        Scheduler.getDefault().addPeriodic(self.periodic)

    def periodic(self):
        if self._last_speed == 0:
            self._leds.setAllBlue()
            return
        error = self.getShooterVelocity() - self._last_speed
        if abs(error) > 2:
            self._leds.setAllRed()
        else:
            self._leds.setAllGreen()

    def getShooterVelocity(self):
        return self._shooter_motor.getEncoderVelocity()

    def getShooterPosition(self):
        return self._shooter_motor.getEncoderPosition()

    def getShooterCurrent(self):
        return self._shooter_motor.getCurrent()

    def isHubConnected(self):
        return self._shooter_motor.isHubConnected()

    def setSpeed(self, speed):
        self._last_speed = speed
        self._shooter_motor.setVelocitySetpoint(speed)

    def setFeed(self, feed):
        error = self.getShooterVelocity() - self._last_speed
        if abs(error) > 2:
            feed = False

        if feed:
            self._left_feeder_servo.set(1.0)
            self._right_feeder_servo.set(1.0)
        else:
            self._left_feeder_servo.set(0)
            self._right_feeder_servo.set(0)

    def getSpinCommand(self):
        """Returns a command that spins the shooter at 40 RPM without feeding."""
        return (
            self.runRepeatedly(lambda: (self.setSpeed(40), self.setFeed(False)))
            .withPriority(Command.DEFAULT_PRIORITY)
            .named("Spin Shooter")
        )

    def getSpinAndFeedCommand(self):
        """Returns a command that spins the shooter and feeds balls."""
        return (
            self.runRepeatedly(lambda: (self.setSpeed(40), self.setFeed(True)))
            .withPriority(Command.DEFAULT_PRIORITY + 1)
            .named("Spin and Feed Shooter")
        )

    def shootTime(self, time):
        """Returns a command that spins and feeds for the given number of seconds."""
        def _shoot(c):
            self.setSpeed(40)
            self.setFeed(True)
            c.wait(time)
            self.setSpeed(0)
            self.setFeed(False)

        return (
            self.run(_shoot)
            .withPriority(Command.DEFAULT_PRIORITY)
            .named(f"Shoot {time}")
        )
