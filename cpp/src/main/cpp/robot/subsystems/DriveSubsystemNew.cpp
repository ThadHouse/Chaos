// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveSubsystemNew.h"

#include <string>

#include <frc/Timer.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/time.h>

#include "../Constants.h"

using namespace frc::robot::subsystems;

// ---------------------------------------------------------------------------
// Private helper
// ---------------------------------------------------------------------------

void DriveSubsystemNew::SetPids(frc::robot::hardware::ExpansionHubMotor& motor) {
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

  frc::Wait(0.5_s);

  m_pinpoint.Update();

  frc::MecanumDriveWheelSpeeds zeroSpeeds{};
  SetDefaultCommand(
      Run([this, zeroSpeeds] { SetSpeeds(zeroSpeeds); }).WithName("Drive Default"));
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void DriveSubsystemNew::Periodic() {
  m_pinpoint.Update();
}

units::ampere_t DriveSubsystemNew::GetFrontLeftCurrent() {
  return m_frontLeftMotor.GetCurrent();
}

units::ampere_t DriveSubsystemNew::GetFrontRightCurrent() {
  return m_frontRightMotor.GetCurrent();
}

units::ampere_t DriveSubsystemNew::GetRearLeftCurrent() {
  return m_rearLeftMotor.GetCurrent();
}

units::ampere_t DriveSubsystemNew::GetRearRightCurrent() {
  return m_rearRightMotor.GetCurrent();
}

bool DriveSubsystemNew::IsHubConnected() {
  return m_frontLeftMotor.IsHubConnected();
}

frc::Pose2d DriveSubsystemNew::GetPose() {
  return m_pinpoint.GetPosition();
}

void DriveSubsystemNew::ResetOdometry(const frc::Pose2d& pose) {
  m_pinpoint.SetPosition(pose);
}

void DriveSubsystemNew::SetSpeeds(const frc::MecanumDriveWheelSpeeds& speeds) {
  m_frontLeftMotor.SetVelocitySetpoint(speeds.frontLeft.value());
  m_frontRightMotor.SetVelocitySetpoint(speeds.frontRight.value());
  m_rearLeftMotor.SetVelocitySetpoint(speeds.rearLeft.value());
  m_rearRightMotor.SetVelocitySetpoint(speeds.rearRight.value());
}

void DriveSubsystemNew::Drive(units::meters_per_second_t xSpeed,
                               units::meters_per_second_t ySpeed,
                               units::radians_per_second_t rot,
                               bool fieldRelative) {
  frc::ChassisSpeeds chassisSpeeds{xSpeed, ySpeed, rot};
  if (fieldRelative) {
    chassisSpeeds =
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassisSpeeds, GetHeading());
  }
  chassisSpeeds = frc::ChassisSpeeds::Discretize(chassisSpeeds, 0.02_s);
  auto mecanumStates =
      DriveConstants::kDriveKinematics.ToWheelSpeeds(chassisSpeeds);
  mecanumStates.Desaturate(DriveConstants::kMaxWheelSpeed);
  SetSpeeds(mecanumStates);
}

void DriveSubsystemNew::DriveJoysticks(double xSpeed, double ySpeed, double rot,
                                        bool fieldRelative) {
  auto xSpeedDelivered = DriveConstants::kMaxSpeed * xSpeed;
  auto ySpeedDelivered = DriveConstants::kMaxSpeed * ySpeed;
  auto rotDelivered = DriveConstants::kMaxAngularSpeed * rot;

  Drive(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
}

frc::MecanumDriveWheelSpeeds DriveSubsystemNew::GetCurrentWheelSpeeds() {
  return frc::MecanumDriveWheelSpeeds{
      units::meters_per_second_t{m_frontLeftMotor.GetEncoderVelocity()},
      units::meters_per_second_t{m_frontRightMotor.GetEncoderVelocity()},
      units::meters_per_second_t{m_rearLeftMotor.GetEncoderVelocity()},
      units::meters_per_second_t{m_rearRightMotor.GetEncoderVelocity()}};
}

frc::MecanumDriveWheelPositions DriveSubsystemNew::GetCurrentWheelDistances() {
  return frc::MecanumDriveWheelPositions{
      units::meter_t{m_frontLeftMotor.GetEncoderPosition()},
      units::meter_t{m_frontRightMotor.GetEncoderPosition()},
      units::meter_t{m_rearLeftMotor.GetEncoderPosition()},
      units::meter_t{m_rearRightMotor.GetEncoderPosition()}};
}

units::radians_per_second_t DriveSubsystemNew::GetTurnRate() {
  return m_pinpoint.GetHeadingVelocity();
}

frc::Rotation2d DriveSubsystemNew::GetHeading() {
  return m_pinpoint.GetHeading();
}

frc2::CommandPtr DriveSubsystemNew::GetJoystickDriveCommand(
    frc::GenericHID& gamepad) {
  return Run([this, &gamepad] {
           DriveJoysticks(-gamepad.GetRawAxis(1), -gamepad.GetRawAxis(0),
                          -gamepad.GetRawAxis(4), true);
         }).WithName("Joystick Drive");
}

frc2::CommandPtr DriveSubsystemNew::DriveForwardTime(double time) {
  return RunEnd(
             [this] { DriveJoysticks(0.0, 1.0, 0.0, true); },
             [this] { DriveJoysticks(0.0, 0.0, 0.0, true); })
      .WithTimeout(units::second_t{time})
      .WithName("Drive Forward " + std::to_string(time));
}
