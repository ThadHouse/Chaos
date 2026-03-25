// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/driverstation/GenericHID.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/kinematics/MecanumDriveWheelPositions.hpp>
#include <wpi/math/kinematics/MecanumDriveWheelVelocities.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/velocity.hpp>

#include "../Constants.h"
#include <wpi/hardware/expansionhub/ExpansionHubMotor.hpp>
#include "GoBildaPinpoint.h"

namespace wpi {
namespace robot {
namespace subsystems {

/**
 * Mecanum drive subsystem with GoBilda Pinpoint odometry.
 *
 * Controls four ExpansionHub motors (hub 0) in a mecanum configuration,
 * using the GoBilda Pinpoint I2C odometry computer for global pose tracking.
 */
class DriveSubsystemNew : public wpi::cmd::SubsystemBase {
 public:
  DriveSubsystemNew();

  /** Called periodically to update odometry. */
  void Periodic() override;

  /** @return Current draw of the front-left motor. */
  wpi::units::ampere_t GetFrontLeftCurrent();

  /** @return Current draw of the front-right motor. */
  wpi::units::ampere_t GetFrontRightCurrent();

  /** @return Current draw of the rear-left motor. */
  wpi::units::ampere_t GetRearLeftCurrent();

  /** @return Current draw of the rear-right motor. */
  wpi::units::ampere_t GetRearRightCurrent();

  /** @return Whether expansion hub 0 is connected. */
  bool IsHubConnected();

  /** @return Current estimated robot pose. */
  wpi::math::Pose2d GetPose();

  /**
   * Resets odometry to the specified pose.
   *
   * @param pose New robot pose.
   */
  void ResetOdometry(const wpi::math::Pose2d& pose);

  /**
   * Sets wheel speeds directly.
   *
   * @param speeds Target wheel speeds.
   */
  void SetSpeeds(const wpi::math::MecanumDriveWheelVelocities& speeds);

  /**
   * Drives the robot at given chassis speeds, optionally field-relative.
   *
   * @param xSpeed        Forward speed.
   * @param ySpeed        Lateral (strafe) speed.
   * @param rot           Angular rate.
   * @param fieldRelative True to interpret x/y speeds as field-relative.
   */
  void Drive(wpi::units::meters_per_second_t xSpeed,
             wpi::units::meters_per_second_t ySpeed, wpi::units::radians_per_second_t rot,
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
  wpi::math::MecanumDriveWheelVelocities GetCurrentWheelSpeeds();

  /** @return Current wheel distance measurements. */
  wpi::math::MecanumDriveWheelPositions GetCurrentWheelDistances();

  /** @return Current heading angular velocity. */
  wpi::units::radians_per_second_t GetTurnRate();

  /** @return Current robot heading. */
  wpi::math::Rotation2d GetHeading();

  /**
   * @return Command that drives the robot using joystick input.
   *
   * @param gamepad Gamepad to read joystick values from.
   */
  wpi::cmd::CommandPtr GetJoystickDriveCommand(wpi::GenericHID& gamepad);

  /**
   * @return Command that drives forward at full speed for the given duration.
   *
   * @param time Duration in seconds.
   */
  wpi::cmd::CommandPtr DriveForwardTime(double time);

 private:
  wpi::ExpansionHubMotor m_frontLeftMotor{
      0, DriveConstants::kFrontLeftMotorPort};
  wpi::ExpansionHubMotor m_frontRightMotor{
      0, DriveConstants::kFrontRightMotorPort};
  wpi::ExpansionHubMotor m_rearLeftMotor{
      0, DriveConstants::kRearLeftMotorPort};
  wpi::ExpansionHubMotor m_rearRightMotor{
      0, DriveConstants::kRearRightMotorPort};

  GoBildaPinpoint m_pinpoint{wpi::I2C::Port::kPort1};

  static void SetPids(wpi::ExpansionHubMotor& motor);
};

}  // namespace subsystems
}  // namespace robot
}  // namespace wpi
