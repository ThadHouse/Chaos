# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
from commands3 import Mechanism, Command, Scheduler
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, MecanumDriveWheelSpeeds, MecanumDriveWheelPositions
from subsystems.go_bilda_pinpoint import GoBildaPinpoint
from constants import DriveConstants


class DriveSubsystemNew(Mechanism):
    """Mecanum drive subsystem using REV Expansion Hub motors and GoBILDA Pinpoint odometry."""

    def __init__(self):
        super().__init__()

        self._front_left_motor = wpilib.ExpansionHubMotor(0, DriveConstants.kFrontLeftMotorPort)
        self._front_right_motor = wpilib.ExpansionHubMotor(0, DriveConstants.kFrontRightMotorPort)
        self._rear_left_motor = wpilib.ExpansionHubMotor(0, DriveConstants.kRearLeftMotorPort)
        self._rear_right_motor = wpilib.ExpansionHubMotor(0, DriveConstants.kRearRightMotorPort)

        self._pinpoint = GoBildaPinpoint(wpilib.I2C.Port.kPort1)

        self._pinpoint.resetPosAndIMU()

        self._pinpoint.setEncoderResolution(GoBildaPinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        self._pinpoint.setOffsets(DriveConstants.kXOffset * 1000, DriveConstants.kYOffset * 1000)
        self._pinpoint.setEncoderDirections(
            GoBildaPinpoint.EncoderDirection.FORWARD,
            GoBildaPinpoint.EncoderDirection.REVERSED,
        )

        self._front_left_motor.setReversed(DriveConstants.kFrontLeftEncoderReversed)
        self._rear_left_motor.setReversed(DriveConstants.kRearLeftEncoderReversed)
        self._front_right_motor.setReversed(DriveConstants.kFrontRightEncoderReversed)
        self._rear_right_motor.setReversed(DriveConstants.kRearRightEncoderReversed)

        self._set_pids(self._front_left_motor)
        self._set_pids(self._front_right_motor)
        self._set_pids(self._rear_left_motor)
        self._set_pids(self._rear_right_motor)

        wpilib.Timer.delay(0.5)

        self._pinpoint.update()

        Scheduler.getDefault().addPeriodic(self.periodic)

        zero_speeds = MecanumDriveWheelSpeeds()
        self.setDefaultCommand(
            self.runRepeatedly(lambda: self.setSpeeds(zero_speeds))
            .withPriority(Command.LOWEST_PRIORITY)
            .named("Drive Default")
        )

    def _set_pids(self, motor):
        motor.setDistancePerCount(DriveConstants.kEncoderDistancePerPulse)
        pid_constants = motor.getVelocityPidConstants()
        pid_constants.setPID(DriveConstants.kP, 0, 0)
        pid_constants.setFF(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA)
        motor.resetEncoder()
        motor.setEnabled(True)

    def periodic(self):
        self._pinpoint.update()

    def getFrontLeftCurrent(self):
        return self._front_left_motor.getCurrent()

    def getFrontRightCurrent(self):
        return self._front_right_motor.getCurrent()

    def getRearLeftCurrent(self):
        return self._rear_left_motor.getCurrent()

    def getRearRightCurrent(self):
        return self._rear_right_motor.getCurrent()

    def isHubConnected(self):
        return self._front_left_motor.isHubConnected()

    def getPose(self):
        return self._pinpoint.getPosition()

    def resetOdometry(self, pose: Pose2d):
        self._pinpoint.setPosition(pose)

    def setSpeeds(self, speeds: MecanumDriveWheelSpeeds):
        self._front_left_motor.setVelocitySetpoint(speeds.frontLeft)
        self._front_right_motor.setVelocitySetpoint(speeds.frontRight)
        self._rear_left_motor.setVelocitySetpoint(speeds.rearLeft)
        self._rear_right_motor.setVelocitySetpoint(speeds.rearRight)

    def drive(self, x_speed, y_speed, rot, field_relative):
        """
        Drives the robot at given x, y and theta speeds.

        :param x_speed: Speed of the robot in the x direction (forward/backwards) in m/s.
        :param y_speed: Speed of the robot in the y direction (sideways) in m/s.
        :param rot: Angular rate of the robot in rad/s.
        :param field_relative: Whether the provided speeds are relative to the field.
        """
        chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot)
        if field_relative:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassis_speeds, self.getHeading()
            )
        chassis_speeds = ChassisSpeeds.discretize(chassis_speeds, 0.02)
        mecanum_states = DriveConstants.kDriveKinematics.toWheelSpeeds(chassis_speeds)
        mecanum_states.desaturate(DriveConstants.kMaxWheelSpeed)
        self.setSpeeds(mecanum_states)

    def driveJoysticks(self, x_speed, y_speed, rot, field_relative):
        x_speed_delivered = DriveConstants.kMaxSpeed * x_speed
        y_speed_delivered = DriveConstants.kMaxSpeed * y_speed
        rot_delivered = DriveConstants.kMaxAngularSpeed * rot
        self.drive(x_speed_delivered, y_speed_delivered, rot_delivered, field_relative)

    def getCurrentWheelSpeeds(self):
        return MecanumDriveWheelSpeeds(
            self._front_left_motor.getEncoderVelocity(),
            self._front_right_motor.getEncoderVelocity(),
            self._rear_left_motor.getEncoderVelocity(),
            self._rear_right_motor.getEncoderVelocity(),
        )

    def getCurrentWheelDistances(self):
        return MecanumDriveWheelPositions(
            self._front_left_motor.getEncoderPosition(),
            self._front_right_motor.getEncoderPosition(),
            self._rear_left_motor.getEncoderPosition(),
            self._rear_right_motor.getEncoderPosition(),
        )

    def getTurnRate(self):
        """Returns the robot's turn rate in rad/s."""
        return self._pinpoint.getHeadingVelocity()

    def getHeading(self):
        """Returns the robot's heading as a Rotation2d."""
        return self._pinpoint.getHeading()

    def getJoystickDriveCommand(self, gamepad):
        """Returns a command to drive the robot with joystick input from the given gamepad."""
        return (
            self.runRepeatedly(lambda: self.driveJoysticks(
                -gamepad.getLeftY(),
                -gamepad.getLeftX(),
                -gamepad.getRightX(),
                True,
            ))
            .withPriority(Command.DEFAULT_PRIORITY)
            .named("Joystick Drive")
        )

    def driveForwardTime(self, time):
        """Returns a command to drive forward for the given number of seconds."""
        def _drive(c):
            self.driveJoysticks(0.0, 1.0, 0.0, True)
            c.wait(time)
            self.driveJoysticks(0.0, 0.0, 0.0, True)

        return (
            self.run(_drive)
            .withPriority(Command.DEFAULT_PRIORITY)
            .named(f"Drive Forward {time}")
        )
