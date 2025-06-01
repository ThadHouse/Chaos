// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Meter;
// import static edu.wpi.first.units.Units.MetersPerSecond;
// import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
// import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class DriveSubsystemNew extends SubsystemBase {
  @NotLogged
  private final ServoHubMotorController m_hubMotors = new ServoHubMotorController();

  private final OctoQuadEncoders m_encoders = new OctoQuadEncoders(Port.kPort0);

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
    m_encoders.resetPositions();

    Timer.delay(0.5);

    m_pinpoint.update();

    m_odometry = new MecanumDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        new MecanumDriveWheelPositions());

    // routine = new SysIdRoutine(
    //     new SysIdRoutine.Config(),
    //     new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));
  }

  // private final SysIdRoutine routine;

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return routine.dynamic(direction);
  // }

  // private void voltageDrive(Voltage voltage) {
  //   var battery = RobotController.getBatteryVoltage();
  //   m_hubMotors.setFrontLeft((voltage.in(Volt)) / battery);
  //   m_hubMotors.setFrontRight((voltage.in(Volt)) / battery);
  //   m_hubMotors.setRearLeft((voltage.in(Volt)) / battery);
  //   m_hubMotors.setRearRight((voltage.in(Volt)) / battery);
  //   lastVoltage = voltage; 
  // }

  // private Voltage lastVoltage = Volt.of(0);

  // private void logMotors(SysIdRoutineLog log) {
  //   log.motor("drive-front-left")
  //       .voltage(lastVoltage)
  //       .linearPosition(Meter.of(m_encoders.getFrontLeftPosition()))
  //       .linearVelocity(MetersPerSecond.of(m_encoders.getFrontLeftRate()));

  //   log.motor("drive-front-right")
  //       .voltage(lastVoltage)
  //       .linearPosition(Meter.of(m_encoders.getFrontRightPosition()))
  //       .linearVelocity(MetersPerSecond.of(m_encoders.getFrontRightRate()));

  //   log.motor("drive-rear-left")
  //       .voltage(lastVoltage)
  //       .linearPosition(Meter.of(m_encoders.getRearLeftPosition()))
  //       .linearVelocity(MetersPerSecond.of(m_encoders.getRearLeftRate()));

  //   log.motor("drive-rear-right")
  //       .voltage(lastVoltage)
  //       .linearPosition(Meter.of(m_encoders.getRearRightPosition()))
  //       .linearVelocity(MetersPerSecond.of(m_encoders.getRearRightRate()));
  // }

  @Override
  public void periodic() {
    m_encoders.periodic();
    m_pinpoint.update();
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

  public Pose2d getPinpointPose() {
    return m_pinpoint.getPosition();
  }

  public Rotation2d getRotation2d() {
    return m_pinpoint.getHeading();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetToZero() {
    m_pinpoint.setPosition(Pose2d.kZero);
    m_pinpoint.update();
    m_odometry = new MecanumDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        new MecanumDriveWheelPositions());
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {

    SmartDashboard.putNumber("frontLeft", speeds.frontLeft);
    SmartDashboard.putNumber("frontRight", speeds.rearRight);
    SmartDashboard.putNumber("readLeft", speeds.rearLeft);
    SmartDashboard.putNumber("rearRight", speeds.rearRight);

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

    SmartDashboard.putNumber("frontLeftOutput", frontLeftOutput);
    SmartDashboard.putNumber("frontRightOutput", frontRightOutput);
    SmartDashboard.putNumber("readLeftOutput", backLeftOutput);
    SmartDashboard.putNumber("rearRightOutput", backRightOutput);

    SmartDashboard.putNumber("frontLeftFF", frontLeftFeedforward);
    SmartDashboard.putNumber("frontRightFF", frontRightFeedforward);
    SmartDashboard.putNumber("readLeftFF", backLeftFeedforward);
    SmartDashboard.putNumber("rearRightFF", backRightFeedforward);

    var battery = RobotController.getBatteryVoltage();
    m_hubMotors.setFrontLeft((frontLeftOutput + frontLeftFeedforward) / battery);
    m_hubMotors.setFrontRight((frontRightOutput + frontRightFeedforward) / battery);
    m_hubMotors.setRearLeft((backLeftOutput + backLeftFeedforward) / battery);
    m_hubMotors.setRearRight((backRightOutput + backRightFeedforward) / battery);
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

    SmartDashboard.putNumber("xSpeed", xSpeedDelivered);
    SmartDashboard.putNumber("ySpeed", ySpeedDelivered);
    SmartDashboard.putNumber("rot", rotDelivered);

    var chassisSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    if (fieldRelative) {
      chassisSpeeds = chassisSpeeds.toRobotRelative(getRotation2d());
    }
    setSpeeds(DriveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds.discretize(0.02))
        .desaturate(DriveConstants.kMaxSpeedMetersPerSecond));
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
}
