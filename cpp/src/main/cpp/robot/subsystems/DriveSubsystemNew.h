// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/GenericHID.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/MecanumDriveWheelPositions.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/velocity.h>

#include "../Constants.h"
#include "../hardware/ExpansionHubMotor.h"
#include "GoBildaPinpoint.h"

namespace frc {
namespace robot {
namespace subsystems {

/**
 * Mecanum drive subsystem with GoBilda Pinpoint odometry.
 *
 * Controls four ExpansionHub motors (hub 0) in a mecanum configuration,
 * using the GoBilda Pinpoint I2C odometry computer for global pose tracking.
 */
class DriveSubsystemNew : public frc2::SubsystemBase {
 public:
  DriveSubsystemNew();

  /** Called periodically to update odometry. */
  void Periodic() override;

  /** @return Current draw of the front-left motor. */
  units::ampere_t GetFrontLeftCurrent();

  /** @return Current draw of the front-right motor. */
  units::ampere_t GetFrontRightCurrent();

  /** @return Current draw of the rear-left motor. */
  units::ampere_t GetRearLeftCurrent();

  /** @return Current draw of the rear-right motor. */
  units::ampere_t GetRearRightCurrent();

  /** @return Whether expansion hub 0 is connected. */
  bool IsHubConnected();

  /** @return Current estimated robot pose. */
  frc::Pose2d GetPose();

  /**
   * Resets odometry to the specified pose.
   *
   * @param pose New robot pose.
   */
  void ResetOdometry(const frc::Pose2d& pose);

  /**
   * Sets wheel speeds directly.
   *
   * @param speeds Target wheel speeds.
   */
  void SetSpeeds(const frc::MecanumDriveWheelSpeeds& speeds);

  /**
   * Drives the robot at given chassis speeds, optionally field-relative.
   *
   * @param xSpeed        Forward speed.
   * @param ySpeed        Lateral (strafe) speed.
   * @param rot           Angular rate.
   * @param fieldRelative True to interpret x/y speeds as field-relative.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  /**
   * Drives using joystick-scaled values in [-1, 1].
   *
   * @param xSpeed        Normalized forward speed.
   * @param ySpeed        Normalized lateral speed.
   * @param rot           Normalized rotation rate.
   * @param fieldRelative True for field-relative control.
   */
  void DriveJoysticks(double xSpeed, double ySpeed, double rot,
                      bool fieldRelative);

  /** @return Current wheel speeds. */
  frc::MecanumDriveWheelSpeeds GetCurrentWheelSpeeds();

  /** @return Current wheel distance measurements. */
  frc::MecanumDriveWheelPositions GetCurrentWheelDistances();

  /** @return Current heading angular velocity. */
  units::radians_per_second_t GetTurnRate();

  /** @return Current robot heading. */
  frc::Rotation2d GetHeading();

  /**
   * @return Command that drives the robot using joystick input.
   *
   * @param gamepad Gamepad to read joystick values from.
   */
  frc2::CommandPtr GetJoystickDriveCommand(frc::GenericHID& gamepad);

  /**
   * @return Command that drives forward at full speed for the given duration.
   *
   * @param time Duration in seconds.
   */
  frc2::CommandPtr DriveForwardTime(double time);

 private:
  frc::robot::hardware::ExpansionHubMotor m_frontLeftMotor{
      0, DriveConstants::kFrontLeftMotorPort};
  frc::robot::hardware::ExpansionHubMotor m_frontRightMotor{
      0, DriveConstants::kFrontRightMotorPort};
  frc::robot::hardware::ExpansionHubMotor m_rearLeftMotor{
      0, DriveConstants::kRearLeftMotorPort};
  frc::robot::hardware::ExpansionHubMotor m_rearRightMotor{
      0, DriveConstants::kRearRightMotorPort};

  GoBildaPinpoint m_pinpoint{frc::I2C::Port::kMXP};

  static void SetPids(frc::robot::hardware::ExpansionHubMotor& motor);
};

}  // namespace subsystems
}  // namespace robot
}  // namespace frc
