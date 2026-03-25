// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveSubsystemNew.h"

#include <string>

#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/system/Timer.hpp>
#include <wpi/units/time.hpp>

#include "../Constants.h"

using namespace wpi::robot::subsystems;
using namespace wpi::units::literals;

// ---------------------------------------------------------------------------
// Private helper
// ---------------------------------------------------------------------------

void DriveSubsystemNew::SetPids(wpi::ExpansionHubMotor& motor) {
  motor.SetDistancePerCount(DriveConstants::kEncoderDistancePerPulse);

  auto& pidConstants = motor.GetVelocityPidConstants();
  pidConstants.SetPID(DriveConstants::kP, 0.0, 0.0);
  pidConstants.SetFF(DriveConstants::kS, DriveConstants::kV,
                     DriveConstants::kA);

  motor.ResetEncoder();
  motor.SetEnabled(true);
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

DriveSubsystemNew::DriveSubsystemNew() {
  m_pinpoint.ResetPosAndIMU();
  m_pinpoint.SetEncoderResolution(
      GoBildaPinpoint::GoBildaOdometryPods::goBILDA_4_BAR_POD);
  m_pinpoint.SetOffsets(DriveConstants::kXOffset, DriveConstants::kYOffset);
  m_pinpoint.SetEncoderDirections(GoBildaPinpoint::EncoderDirection::FORWARD,
                                   GoBildaPinpoint::EncoderDirection::REVERSED);

  m_frontLeftMotor.SetReversed(DriveConstants::kFrontLeftEncoderReversed);
  m_rearLeftMotor.SetReversed(DriveConstants::kRearLeftEncoderReversed);
  m_frontRightMotor.SetReversed(DriveConstants::kFrontRightEncoderReversed);
  m_rearRightMotor.SetReversed(DriveConstants::kRearRightEncoderReversed);

  SetPids(m_frontLeftMotor);
  SetPids(m_frontRightMotor);
  SetPids(m_rearLeftMotor);
  SetPids(m_rearRightMotor);

  wpi::Wait(0.5_s);

  m_pinpoint.Update();

  wpi::math::MecanumDriveWheelVelocities zeroSpeeds{};
  SetDefaultCommand(
      Run([this, zeroSpeeds] { SetSpeeds(zeroSpeeds); }).WithName("Drive Default"));
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void DriveSubsystemNew::Periodic() {
  m_pinpoint.Update();
}

wpi::units::ampere_t DriveSubsystemNew::GetFrontLeftCurrent() {
  return m_frontLeftMotor.GetCurrent();
}

wpi::units::ampere_t DriveSubsystemNew::GetFrontRightCurrent() {
  return m_frontRightMotor.GetCurrent();
}

wpi::units::ampere_t DriveSubsystemNew::GetRearLeftCurrent() {
  return m_rearLeftMotor.GetCurrent();
}

wpi::units::ampere_t DriveSubsystemNew::GetRearRightCurrent() {
  return m_rearRightMotor.GetCurrent();
}

bool DriveSubsystemNew::IsHubConnected() {
  return m_frontLeftMotor.IsHubConnected();
}

wpi::math::Pose2d DriveSubsystemNew::GetPose() {
  return m_pinpoint.GetPosition();
}

void DriveSubsystemNew::ResetOdometry(const wpi::math::Pose2d& pose) {
  m_pinpoint.SetPosition(pose);
}

void DriveSubsystemNew::SetSpeeds(const wpi::math::MecanumDriveWheelVelocities& speeds) {
  m_frontLeftMotor.SetVelocitySetpoint(speeds.frontLeft.value());
  m_frontRightMotor.SetVelocitySetpoint(speeds.frontRight.value());
  m_rearLeftMotor.SetVelocitySetpoint(speeds.rearLeft.value());
  m_rearRightMotor.SetVelocitySetpoint(speeds.rearRight.value());
}

void DriveSubsystemNew::Drive(wpi::units::meters_per_second_t xSpeed,
                               wpi::units::meters_per_second_t ySpeed,
                               wpi::units::radians_per_second_t rot,
                               bool fieldRelative) {
  wpi::math::ChassisVelocities chassisVelocities{xSpeed, ySpeed, rot};
  if (fieldRelative) {
    chassisVelocities = chassisVelocities.ToRobotRelative(GetHeading());
  }
  chassisVelocities = chassisVelocities.Discretize(0.02_s);
  auto mecanumStates =
      DriveConstants::kDriveKinematics.ToWheelVelocities(chassisVelocities);
  mecanumStates = mecanumStates.Desaturate(DriveConstants::kMaxWheelSpeed);
  SetSpeeds(mecanumStates);
}

void DriveSubsystemNew::DriveJoysticks(double xSpeed, double ySpeed, double rot,
                                        bool fieldRelative) {
  auto xSpeedDelivered = DriveConstants::kMaxSpeed * xSpeed;
  auto ySpeedDelivered = DriveConstants::kMaxSpeed * ySpeed;
  auto rotDelivered = DriveConstants::kMaxAngularSpeed * rot;

  Drive(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
}

wpi::math::MecanumDriveWheelVelocities DriveSubsystemNew::GetCurrentWheelSpeeds() {
  return wpi::math::MecanumDriveWheelVelocities{
      wpi::units::meters_per_second_t{m_frontLeftMotor.GetEncoderVelocity()},
      wpi::units::meters_per_second_t{m_frontRightMotor.GetEncoderVelocity()},
      wpi::units::meters_per_second_t{m_rearLeftMotor.GetEncoderVelocity()},
      wpi::units::meters_per_second_t{m_rearRightMotor.GetEncoderVelocity()}};
}

wpi::math::MecanumDriveWheelPositions DriveSubsystemNew::GetCurrentWheelDistances() {
  return wpi::math::MecanumDriveWheelPositions{
      wpi::units::meter_t{m_frontLeftMotor.GetEncoderPosition()},
      wpi::units::meter_t{m_frontRightMotor.GetEncoderPosition()},
      wpi::units::meter_t{m_rearLeftMotor.GetEncoderPosition()},
      wpi::units::meter_t{m_rearRightMotor.GetEncoderPosition()}};
}

wpi::units::radians_per_second_t DriveSubsystemNew::GetTurnRate() {
  return m_pinpoint.GetHeadingVelocity();
}

wpi::math::Rotation2d DriveSubsystemNew::GetHeading() {
  return m_pinpoint.GetHeading();
}

wpi::cmd::CommandPtr DriveSubsystemNew::GetJoystickDriveCommand(
    wpi::GenericHID& gamepad) {
  return Run([this, &gamepad] {
           DriveJoysticks(-gamepad.GetRawAxis(1), -gamepad.GetRawAxis(0),
                          -gamepad.GetRawAxis(4), true);
         }).WithName("Joystick Drive");
}

wpi::cmd::CommandPtr DriveSubsystemNew::DriveForwardTime(double time) {
  return RunEnd(
             [this] { DriveJoysticks(0.0, 1.0, 0.0, true); },
             [this] { DriveJoysticks(0.0, 0.0, 0.0, true); })
      .WithTimeout(wpi::units::second_t{time})
      .WithName("Drive Forward " + std::to_string(time));
}
