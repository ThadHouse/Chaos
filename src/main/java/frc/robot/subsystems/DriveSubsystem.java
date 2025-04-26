// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class DriveSubsystem extends SubsystemBase {
  @NotLogged
  private final ServoHubMotorController m_hubMotors = new ServoHubMotorController();

  private final MecanumDrive m_drive = new MecanumDrive(m_hubMotors::setFrontLeft, m_hubMotors::setRearLeft,
      m_hubMotors::setFrontRight, m_hubMotors::setRearRight);

  private final OctoQuadEncoders m_encoders = new OctoQuadEncoders();

  // Odometry class for tracking robot pose
  @NotLogged
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
      DriveConstants.kDriveKinematics,
      getRotation2d(),
      new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    m_encoders.periodic();
    // Update the odometry in the periodic block
    m_odometry.update(getRotation2d(), getCurrentWheelDistances());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPose();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, getRotation2d());
    } else {
      m_drive.driveCartesian(xSpeed, ySpeed, rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(
      double frontLeftVoltage,
      double frontRightVoltage,
      double rearLeftVoltage,
      double rearRightVoltage) {
    m_hubMotors.setFrontLeft(frontLeftVoltage / 12.0);
    m_hubMotors.setRearLeft(rearLeftVoltage / 12.0);
    m_hubMotors.setFrontRight(frontRightVoltage / 12.0);
    m_hubMotors.setRearRight(rearRightVoltage / 12.0);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_encoders.resetPositions();
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_encoders.getFrontLeftRate(),
        m_encoders.getFrontRightRate(),
        m_encoders.getRearLeftRate(),
        m_encoders.getRearRightRate());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a
   *         MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_encoders.getFrontLeftPosition(),
        m_encoders.getFrontRightPosition(),
        m_encoders.getRearLeftPosition(),
        m_encoders.getRearRightPosition());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    //m_gyro.reset();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return 0;
    //return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return 0;
    // return -m_gyro.getRate();
  }
}
