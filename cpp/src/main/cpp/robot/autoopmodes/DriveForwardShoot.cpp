// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveForwardShoot.h"

#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/commands2/Commands.hpp>

using namespace wpi::robot::autoopmodes;

DriveForwardShoot::DriveForwardShoot(Robot& robot)
    : m_robot{robot},
      m_command{m_robot.GetDrive()
                    .DriveForwardTime(2)
                    .AndThen(m_robot.GetShooter().ShootTime(5))
                    .WithName("DriveForwardShoot")} {}

void DriveForwardShoot::DisabledPeriodic() {
  m_robot.RobotPeriodic();
}

void DriveForwardShoot::Start() {
  m_command.Schedule();
}

void DriveForwardShoot::Periodic() {
  m_robot.RobotPeriodic();
}

void DriveForwardShoot::End() {
  m_command.Cancel();
}
