// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.GoBildaPinpoint.GoBildaOdometryPods;
import frc.utils.ExpansionHubMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class DriveSubsystemNew extends SubsystemBase {
  private final ExpansionHubMotor m_frontLeftMotor = new ExpansionHubMotor(0, DriveConstants.kFrontLeftMotorPort);
  private final ExpansionHubMotor m_frontRightMotor = new ExpansionHubMotor(0, DriveConstants.kFrontRightMotorPort);
  private final ExpansionHubMotor m_rearLeftMotor = new ExpansionHubMotor(0, DriveConstants.kRearLeftMotorPort);
  private final ExpansionHubMotor m_rearRightMotor = new ExpansionHubMotor(0, DriveConstants.kRearRightMotorPort);

  private final GoBildaPinpoint m_pinpoint = new GoBildaPinpoint(Port.kPort1);

  // Odometry class for tracking robot pose
  @NotLogged
  private MecanumDriveOdometry m_odometry;

  private final PIDController m_frontLeftPIDController = new PIDController(DriveConstants.kPDrive, 0, 0);
  private final PIDController m_frontRightPIDController = new PIDController(DriveConstants.kPDrive, 0, 0);
  private final PIDController m_backLeftPIDController = new PIDController(DriveConstants.kPDrive, 0, 0);
  private final PIDController m_backRightPIDController = new PIDController(DriveConstants.kPDrive, 0, 0);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemNew() {
    m_pinpoint.resetPosAndIMU();

    m_pinpoint.setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);
    // TODO set x and y on pinpoint

    m_frontLeftMotor.resetEncoder();
    m_frontRightMotor.resetEncoder();
    m_rearLeftMotor.resetEncoder();
    m_rearRightMotor.resetEncoder();

    m_frontLeftMotor.setDistancePerCount(DriveConstants.kEncoderDistancePerPulse);
    m_frontRightMotor.setDistancePerCount(DriveConstants.kEncoderDistancePerPulse);
    m_rearLeftMotor.setDistancePerCount(DriveConstants.kEncoderDistancePerPulse);
    m_rearRightMotor.setDistancePerCount(DriveConstants.kEncoderDistancePerPulse);

    m_frontLeftMotor.setReversed(DriveConstants.kFrontLeftEncoderReversed);
    m_rearLeftMotor.setReversed(DriveConstants.kRearLeftEncoderReversed);
    m_frontRightMotor.setReversed(DriveConstants.kFrontRightEncoderReversed);
    m_rearRightMotor.setReversed(DriveConstants.kRearRightEncoderReversed);

    m_frontLeftMotor.setEnabled(true);
    m_frontRightMotor.setEnabled(true);
    m_rearLeftMotor.setEnabled(true);
    m_rearRightMotor.setEnabled(true);

    Timer.delay(0.5);

    m_pinpoint.setPosition(Pose2d.kZero.rotateBy(Rotation2d.kPi));

    m_pinpoint.update();

    m_odometry = new MecanumDriveOdometry(
        DriveConstants.kDriveKinematics,
        m_pinpoint.getHeading(),
        new MecanumDriveWheelPositions());
  }

  @Override
  public void periodic() {
    m_pinpoint.update();
    // Update the odometry in the periodic block
    m_odometry.update(m_pinpoint.getHeading(), getCurrentWheelDistances());
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

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPose();
  }

  public Pose2d getPinpointPose() {
    return m_pinpoint.getPosition();
  }

  // Reset the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    m_pinpoint.setPosition(pose);
    m_odometry.resetPosition(
        m_pinpoint.getHeading(),
        getCurrentWheelDistances(),
        pose);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {

    final double frontLeftFeedforward = DriveConstants.kFeedforward.calculate(speeds.frontLeft, speeds.frontLeft);
    final double frontRightFeedforward = DriveConstants.kFeedforward.calculate(speeds.frontRight, speeds.frontRight);
    final double backLeftFeedforward = DriveConstants.kFeedforward.calculate(speeds.rearLeft, speeds.rearLeft);
    final double backRightFeedforward = DriveConstants.kFeedforward.calculate(speeds.rearRight, speeds.rearRight);

    MecanumDriveWheelSpeeds currentSpeeds = getCurrentWheelSpeeds();

    final double frontLeftOutput = m_frontLeftPIDController.calculate(
        currentSpeeds.frontLeft, speeds.frontLeft);
    final double frontRightOutput = m_frontRightPIDController.calculate(
        currentSpeeds.frontRight, speeds.frontRight);
    final double backLeftOutput = m_backLeftPIDController.calculate(
        currentSpeeds.rearLeft, speeds.rearLeft);
    final double backRightOutput = m_backRightPIDController.calculate(
        currentSpeeds.rearRight, speeds.rearRight);

    m_frontLeftMotor.setVoltage(Volt.of(frontLeftOutput + frontLeftFeedforward));
    m_frontRightMotor.setVoltage(Volt.of(frontRightOutput + frontRightFeedforward));
    m_rearLeftMotor.setVoltage(Volt.of(backLeftOutput + backLeftFeedforward));
    m_rearRightMotor.setVoltage(Volt.of(backRightOutput + backRightFeedforward));
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
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var chassisSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    if (fieldRelative) {
      chassisSpeeds = chassisSpeeds.toRobotRelative(m_odometry.getPose().getRotation());
    }
    chassisSpeeds = chassisSpeeds.discretize(0.02);
    var mecanumStates = DriveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);
    mecanumStates = mecanumStates.desaturate(DriveConstants.kMaxWheelSpeedMetersPerSecond);
    setSpeeds(mecanumStates);
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

  public double getHeading() {
    return m_odometry.getPose().getRotation().getRadians();
  }

  public double getPinpointTurnRate() {
    return m_pinpoint.getHeadingVelocity().in(RadiansPerSecond);
  }

  public double getPinpointHeading() {
    return m_pinpoint.getHeading().getRadians();
  }

  public Rotation2d getPinpointRotation2d() {
    return m_pinpoint.getHeading();
  }

  @NotLogged
  private Voltage m_lastVoltage = Volts.of(0);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              voltage -> {
                m_frontLeftMotor.setVoltage(voltage);
                m_frontRightMotor.setVoltage(voltage);
                m_rearLeftMotor.setVoltage(voltage);
                m_rearRightMotor.setVoltage(voltage);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                log.motor("drive-left-front")
                    .voltage(m_lastVoltage)
                    .linearPosition(Meters.of(m_frontLeftMotor.getEncoderPosition()))
                    .linearVelocity(MetersPerSecond.of(m_frontLeftMotor.getEncoderVelocity()));
                log.motor("drive-left-rear")
                    .voltage(m_lastVoltage)
                    .linearPosition(Meters.of(m_rearLeftMotor.getEncoderPosition()))
                    .linearVelocity(MetersPerSecond.of(m_rearLeftMotor.getEncoderVelocity()));
                log.motor("drive-right-front")
                    .voltage(m_lastVoltage)
                    .linearPosition(Meters.of(m_frontRightMotor.getEncoderPosition()))
                    .linearVelocity(MetersPerSecond.of(m_frontRightMotor.getEncoderVelocity()));
                log.motor("drive-right-rear")
                    .voltage(m_lastVoltage)
                    .linearPosition(Meters.of(m_rearRightMotor.getEncoderPosition()))
                    .linearVelocity(MetersPerSecond.of(m_rearRightMotor.getEncoderVelocity()));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
