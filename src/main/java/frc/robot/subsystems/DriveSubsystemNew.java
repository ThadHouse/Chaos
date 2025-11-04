// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.wpilib.commands3.Mechanism;
import org.wpilib.commands3.Scheduler;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ExpansionHubMotor;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.GoBildaPinpoint.EncoderDirection;
import frc.robot.subsystems.GoBildaPinpoint.GoBildaOdometryPods;

@Logged
public class DriveSubsystemNew extends Mechanism {
  private final ExpansionHubMotor m_frontLeftMotor = new ExpansionHubMotor(0, DriveConstants.kFrontLeftMotorPort);
  private final ExpansionHubMotor m_frontRightMotor = new ExpansionHubMotor(0, DriveConstants.kFrontRightMotorPort);
  private final ExpansionHubMotor m_rearLeftMotor = new ExpansionHubMotor(0, DriveConstants.kRearLeftMotorPort);
  private final ExpansionHubMotor m_rearRightMotor = new ExpansionHubMotor(0, DriveConstants.kRearRightMotorPort);

  private final GoBildaPinpoint m_pinpoint = new GoBildaPinpoint(Port.kPort1);

  private static void setPids(ExpansionHubMotor motor) {
    motor.setDistancePerCount(DriveConstants.kEncoderDistancePerPulse);

    var pidConstants = motor.getVelocityPidConstants();
    pidConstants.setPID(DriveConstants.kP, 0, 0);
    pidConstants.setFF(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

    motor.resetEncoder();
    motor.setEnabled(true);
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemNew() {
    m_pinpoint.resetPosAndIMU();

    m_pinpoint.setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);
    m_pinpoint.setOffsets(DriveConstants.kXOffset, DriveConstants.kYOffset);
    m_pinpoint.setEncoderDirections(EncoderDirection.FORWARD, EncoderDirection.REVERSED);

    m_frontLeftMotor.setReversed(DriveConstants.kFrontLeftEncoderReversed);
    m_rearLeftMotor.setReversed(DriveConstants.kRearLeftEncoderReversed);
    m_frontRightMotor.setReversed(DriveConstants.kFrontRightEncoderReversed);
    m_rearRightMotor.setReversed(DriveConstants.kRearRightEncoderReversed);

    setPids(m_frontLeftMotor);
    setPids(m_frontRightMotor);
    setPids(m_rearLeftMotor);
    setPids(m_rearRightMotor);

    Timer.delay(0.5);

    m_pinpoint.update();

    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void periodic() {
    m_pinpoint.update();
  }

  public Current getFrontLeftCurrent() {
    return m_frontLeftMotor.getCurrent();
  }

  public Current getFrontRightCurrent() {
    return m_frontRightMotor.getCurrent();
  }

  public Current getRearLeftCurrent() {
    return m_rearLeftMotor.getCurrent();
  }

  public Current getRearRightCurrent() {
    return m_rearRightMotor.getCurrent();
  }

  public boolean isHubConnected() {
    return m_frontLeftMotor.isHubConnected();
  }

  public Pose2d getPose() {
    return m_pinpoint.getPosition();
  }

  // Reset the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    m_pinpoint.setPosition(pose);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {

    m_frontLeftMotor.setVelocitySetpoint(speeds.frontLeft);
    m_frontRightMotor.setVelocitySetpoint(speeds.frontRight);
    m_rearLeftMotor.setVelocitySetpoint(speeds.rearLeft);
    m_rearRightMotor.setVelocitySetpoint(speeds.rearRight);
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
  public void drive(LinearVelocity xSpeed, LinearVelocity ySpeed, AngularVelocity rot, boolean fieldRelative) {

    var chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    if (fieldRelative) {
      chassisSpeeds = chassisSpeeds.toRobotRelative(getHeading());
    }
    chassisSpeeds = chassisSpeeds.discretize(0.02);
    var mecanumStates = DriveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);
    mecanumStates = mecanumStates.desaturate(DriveConstants.kMaxWheelSpeed);
    setSpeeds(mecanumStates);
  }

  public void driveJoysticks(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var xSpeedDelivered = DriveConstants.kMaxSpeed.times(xSpeed);
    var ySpeedDelivered = DriveConstants.kMaxSpeed.times(ySpeed);
    var rotDelivered = DriveConstants.kMaxAngularSpeed.times(rot);

    drive(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftMotor.getEncoderVelocity(),
        m_frontRightMotor.getEncoderVelocity(),
        m_rearLeftMotor.getEncoderVelocity(),
        m_rearRightMotor.getEncoderVelocity());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a
   *         MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftMotor.getEncoderPosition(),
        m_frontRightMotor.getEncoderPosition(),
        m_rearLeftMotor.getEncoderPosition(),
        m_rearRightMotor.getEncoderPosition());
  }

  public AngularVelocity getTurnRate() {
    return m_pinpoint.getHeadingVelocity();
  }

  public Rotation2d getHeading() {
    return m_pinpoint.getHeading();
  }

  // @NotLogged
  // private Voltage m_lastVoltage = Volts.of(0);

  // private final SysIdRoutine m_sysIdRoutine =
  //     new SysIdRoutine(
  //         // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
  //         new SysIdRoutine.Config(),
  //         new SysIdRoutine.Mechanism(
  //             // Tell SysId how to plumb the driving voltage to the motors.
  //             voltage -> {
  //               m_frontLeftMotor.setVoltage(voltage);
  //               m_frontRightMotor.setVoltage(voltage);
  //               m_rearLeftMotor.setVoltage(voltage);
  //               m_rearRightMotor.setVoltage(voltage);
  //             },
  //             // Tell SysId how to record a frame of data for each motor on the mechanism being
  //             // characterized.
  //             log -> {
  //               log.motor("drive-left-front")
  //                   .voltage(m_lastVoltage)
  //                   .linearPosition(Meters.of(m_frontLeftMotor.getEncoderPosition()))
  //                   .linearVelocity(MetersPerSecond.of(m_frontLeftMotor.getEncoderVelocity()));
  //               log.motor("drive-left-rear")
  //                   .voltage(m_lastVoltage)
  //                   .linearPosition(Meters.of(m_rearLeftMotor.getEncoderPosition()))
  //                   .linearVelocity(MetersPerSecond.of(m_rearLeftMotor.getEncoderVelocity()));
  //               log.motor("drive-right-front")
  //                   .voltage(m_lastVoltage)
  //                   .linearPosition(Meters.of(m_frontRightMotor.getEncoderPosition()))
  //                   .linearVelocity(MetersPerSecond.of(m_frontRightMotor.getEncoderVelocity()));
  //               log.motor("drive-right-rear")
  //                   .voltage(m_lastVoltage)
  //                   .linearPosition(Meters.of(m_rearRightMotor.getEncoderPosition()))
  //                   .linearVelocity(MetersPerSecond.of(m_rearRightMotor.getEncoderVelocity()));
  //             },
  //             // Tell SysId to make generated commands require this subsystem, suffix test state in
  //             // WPILog with this subsystem's name ("drive")
  //             this));

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutine.dynamic(direction);
  // }
}
