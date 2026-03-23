# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from wpimath.geometry import Translation2d
from wpimath.kinematics import MecanumDriveKinematics
import wpilib
import math


class DriveConstants:
    kMaxSpeed = 0.6  # meters per second
    kMaxAngularSpeed = 1.0  # radians per second
    kMaxWheelSpeed = 1.0  # meters per second

    kFrontLeftMotorPort = 1
    kRearLeftMotorPort = 0
    kFrontRightMotorPort = 2
    kRearRightMotorPort = 3

    kFrontLeftEncoderReversed = False
    kRearLeftEncoderReversed = False
    kFrontRightEncoderReversed = True
    kRearRightEncoderReversed = True

    kTrackWidth = 0.416  # meters (416 mm)
    kTrackWidthToCenter = kTrackWidth / 2
    # Distance between centers of right and left wheels on robot
    kWheelBase = 0.336  # meters (336 mm) - 14 large holes * 24mm per large hole
    kWheelBaseToCenter = kWheelBase / 2
    # Distance between centers of front and back wheels on robot

    kDriveKinematics = MecanumDriveKinematics(
        Translation2d(kWheelBaseToCenter, kTrackWidthToCenter),    # Front Left
        Translation2d(kWheelBaseToCenter, -kTrackWidthToCenter),   # Front Right
        Translation2d(-kWheelBaseToCenter, kTrackWidthToCenter),   # Rear Left
        Translation2d(-kWheelBaseToCenter, -kTrackWidthToCenter),  # Rear Right
    )

    kEncoderCPR = 537.7
    kWheelDiameterMeters = 0.15
    kEncoderDistancePerPulse = (kWheelDiameterMeters * math.pi) / kEncoderCPR

    kS = 0.43007
    kV = 4.9336
    kA = 0.71773

    kP = 2.7238

    kYOffset = 0.120  # meters (120 mm) - 15 holes * 8mm per hole
    kXOffset = 0.092  # meters (92 mm) - 11.5 holes * 8mm per hole


class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05


class ShooterConstants:
    kShooterMotorPort = 0
    kEncoderPort = 0
    kI2cPort = wpilib.I2C.Port.kPort0
    kP = 0.16666
    kI = 0.0
    kD = 0.0
    kS = 0.56532
    kA = 0.17586
    kV = 0.13448

    kEncoderCPR = 28
    kEncoderDistancePerPulse = 1.0 / kEncoderCPR  # Distance units in rotations


class AutoConstants:
    kMaxSpeedMetersPerSecond = 3.0  # meters per second
    kMaxAccelerationMetersPerSecondSquared = 3.0  # meters per second squared
    kMaxAngularSpeedRadiansPerSecond = math.pi  # radians per second
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi  # radians per second squared

    kPXController = 0.5
    kPYController = 0.5
    kPThetaController = 0.5
