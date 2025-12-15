// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Millimeter;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import org.wpilib.hardware.bus.I2C;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.MecanumDriveKinematics;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;

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
    public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(0.6);
    public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(1);
    public static final LinearVelocity kMaxWheelSpeed = MetersPerSecond.of(1.0);

    public static final int kFrontLeftMotorPort = 1;
    public static final int kRearLeftMotorPort = 0;
    public static final int kFrontRightMotorPort = 2;
    public static final int kRearRightMotorPort = 3;

    public static final boolean kFrontLeftEncoderReversed = false;
    public static final boolean kRearLeftEncoderReversed = false;
    public static final boolean kFrontRightEncoderReversed = true;
    public static final boolean kRearRightEncoderReversed = true;

    public static final Distance kTrackWidth = Millimeter.of(416);
    public static final Distance kTrackWidthToCenter = kTrackWidth.div(2);
    // Distance between centers of right and left wheels on robot
    public static final Distance kWheelBase = Millimeter.of(336); // 14 large holes * 24mm per large hole
    public static final Distance kWheelBaseToCenter = kWheelBase.div(2);
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBaseToCenter, kTrackWidthToCenter),
            new Translation2d(kWheelBaseToCenter, kTrackWidthToCenter.unaryMinus()),
            new Translation2d(kWheelBaseToCenter.unaryMinus(), kTrackWidthToCenter),
            new Translation2d(kWheelBaseToCenter.unaryMinus(), kTrackWidthToCenter.unaryMinus()));

    public static final double kEncoderCPR = 537.7;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

    public static final double kS = 0.43007;
    public static final double kV = 4.9336;
    public static final double kA = 0.71773;

    public static final double kP = 2.7238;

    public static final Distance kYOffset = Millimeter.of(120); // 15 holes * 8mm per hole
    public static final Distance kXOffset = Millimeter.of(92); // 11.5 holes * 8mm per hole
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
  }

  public static final class AutoConstants {
    public static final LinearVelocity kMaxSpeedMetersPerSecond = MetersPerSecond.of(3);
    public static final LinearAcceleration kMaxAccelerationMetersPerSecondSquared = MetersPerSecondPerSecond.of(3);
    public static final AngularVelocity kMaxAngularSpeedRadiansPerSecond = RadiansPerSecond.of( Math.PI);
    public static final AngularAcceleration kMaxAngularSpeedRadiansPerSecondSquared = RadiansPerSecondPerSecond.of(Math.PI);

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;
  }
}
