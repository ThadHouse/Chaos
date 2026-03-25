// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

#include <cmath>
#include <string>

#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/units/time.hpp>

#include "../Constants.h"

using namespace wpi::robot::subsystems;

Shooter::Shooter(Leds& leds) : m_leds{leds} {
  m_shooterMotor.SetReversed(true);
  m_shooterMotor.SetDistancePerCount(ShooterConstants::kEncoderDistancePerPulse);
  m_shooterMotor.SetEnabled(true);

  m_leftFeederServo.SetContinousRotationMode(true);
  m_rightFeederServo.SetContinousRotationMode(true);

  m_leftFeederServo.SetReversed(true);

  m_leftFeederServo.SetEnabled(true);
  m_rightFeederServo.SetEnabled(true);

  auto& pidConstants = m_shooterMotor.GetVelocityPidConstants();
  pidConstants.SetPID(ShooterConstants::kP, ShooterConstants::kI,
                      ShooterConstants::kD);
  pidConstants.SetFF(ShooterConstants::kS, ShooterConstants::kV,
                     ShooterConstants::kA);

  SetDefaultCommand(Run([this] {
    SetSpeed(0);
    SetFeed(false);
  }).WithName("Default Shooter"));
}

void Shooter::Periodic() {
  if (m_lastSpeed == 0.0) {
    m_leds.SetAllBlue();
    return;
  }
  double error = GetShooterVelocity() - m_lastSpeed;
  if (std::abs(error) > 2.0) {
    m_leds.SetAllRed();
  } else {
    m_leds.SetAllGreen();
  }
}

double Shooter::GetShooterVelocity() {
  return m_shooterMotor.GetEncoderVelocity();
}

double Shooter::GetShooterPosition() {
  return m_shooterMotor.GetEncoderPosition();
}

wpi::units::ampere_t Shooter::GetShooterCurrent() {
  return m_shooterMotor.GetCurrent();
}

bool Shooter::IsHubConnected() {
  return m_shooterMotor.IsHubConnected();
}

void Shooter::SetSpeed(double speed) {
  m_lastSpeed = speed;
  m_shooterMotor.SetVelocitySetpoint(speed);
}

void Shooter::SetFeed(bool feed) {
  double error = GetShooterVelocity() - m_lastSpeed;
  if (std::abs(error) > 2.0) {
    feed = false;
  }

  if (feed) {
    m_leftFeederServo.Set(1.0);
    m_rightFeederServo.Set(1.0);
  } else {
    m_leftFeederServo.Set(0.0);
    m_rightFeederServo.Set(0.0);
  }
}

wpi::cmd::CommandPtr Shooter::GetSpinCommand() {
  return Run([this] {
           SetSpeed(40);
           SetFeed(false);
         }).WithName("Spin Shooter");
}

wpi::cmd::CommandPtr Shooter::GetSpinAndFeedCommand() {
  return Run([this] {
           SetSpeed(40);
           SetFeed(true);
         }).WithName("Spin and Feed Shooter");
}

wpi::cmd::CommandPtr Shooter::ShootTime(double time) {
  return RunEnd(
             [this] {
               SetSpeed(40);
               SetFeed(true);
             },
             [this] {
               SetSpeed(0);
               SetFeed(false);
             })
      .WithTimeout(wpi::units::second_t{time})
      .WithName("Shoot " + std::to_string(time));
}
