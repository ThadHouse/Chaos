// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

#include <cmath>
#include <string>

#include <frc/commands3/Command.h>
#include <frc/commands3/Scheduler.h>
#include <units/time.h>

#include "../Constants.h"

using namespace frc::robot::subsystems;

Shooter::Shooter(Leds& leds) : m_leds{leds} {
  m_shooterMotor.SetReversed(true);
  m_shooterMotor.SetDistancePerCount(ShooterConstants::kEncoderDistancePerPulse);
  m_shooterMotor.SetEnabled(true);

  m_leftFeederServo.SetContinuousRotationMode(true);
  m_rightFeederServo.SetContinuousRotationMode(true);

  m_leftFeederServo.SetReversed(true);

  m_leftFeederServo.SetEnabled(true);
  m_rightFeederServo.SetEnabled(true);

  auto pidConstants = m_shooterMotor.GetVelocityPidConstants();
  pidConstants.SetPID(ShooterConstants::kP, ShooterConstants::kI,
                      ShooterConstants::kD);
  pidConstants.SetFF(ShooterConstants::kS, ShooterConstants::kV,
                     ShooterConstants::kA);

  SetDefaultCommand(RunRepeatedly([this] {
                      SetSpeed(0);
                      SetFeed(false);
                    })
                        .WithPriority(frc::commands3::Command::kLowestPriority)
                        .Named("Default Shooter"));

  frc::commands3::Scheduler::GetDefault().AddPeriodic(
      [this] { Periodic(); });
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

units::ampere_t Shooter::GetShooterCurrent() {
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

frc::commands3::Command Shooter::GetSpinCommand() {
  return RunRepeatedly([this] {
           SetSpeed(40);
           SetFeed(false);
         })
      .WithPriority(frc::commands3::Command::kDefaultPriority)
      .Named("Spin Shooter");
}

frc::commands3::Command Shooter::GetSpinAndFeedCommand() {
  return RunRepeatedly([this] {
           SetSpeed(40);
           SetFeed(true);
         })
      .WithPriority(frc::commands3::Command::kDefaultPriority + 1)
      .Named("Spin and Feed Shooter");
}

frc::commands3::Command Shooter::ShootTime(double time) {
  return Run([this, time](auto& c) {
           SetSpeed(40);
           SetFeed(true);
           c.Wait(units::second_t{time});
           SetSpeed(0);
           SetFeed(false);
         })
      .WithPriority(frc::commands3::Command::kDefaultPriority)
      .Named("Shoot " + std::to_string(time));
}
