// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.6;
    public static final double kMaxAngularSpeed = 1; // radians per second
    public static final double kMaxWheelSpeedMetersPerSecond = 1.0;

    public static final int kFrontLeftMotorPort = 1;
    public static final int kRearLeftMotorPort = 0;
    public static final int kFrontRightMotorPort = 2;
    public static final int kRearRightMotorPort = 3;

    public static final boolean kFrontLeftEncoderReversed = false;
    public static final boolean kRearLeftEncoderReversed = false;
    public static final boolean kFrontRightEncoderReversed = true;
    public static final boolean kRearRightEncoderReversed = true;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kEncoderCPR = 537.7;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

    public static final double kEncoderSampleRate = 1000.0 / 50;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(0.43007, 4.9336, 0.71773);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDrive = 2.7238;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorPort = 0;
    public static final int kEncoderPort = 0;
    public static final I2C.Port kI2cPort = I2C.Port.kPort0;
    public static final double kP = 0.16666;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.56532;
    public static final double kA = 0.17586;
    public static final double kV = 0.13448;

    public static final int kEncoderCPR = 28;
    public static final double kEncoderDistancePerPulse = 1.0 / kEncoderCPR; // Distance units in rotations

    public static final double kEncoderSampleRate = 1000.0 / 50;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
