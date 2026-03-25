// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveForwardShoot.h"

#include <frc/commands3/Command.h>
#include <frc/commands3/Scheduler.h>

// Registration macro — equivalent to Java @Autonomous annotation
REGISTER_AUTONOMOUS(frc::robot::autoopmodes::DriveForwardShoot)

using namespace frc::robot::autoopmodes;

DriveForwardShoot::DriveForwardShoot(Robot& robot)
    : m_robot{robot},
      m_command{frc::commands3::Command::Sequence(
                    m_robot.GetDrive().DriveForwardTime(2),
                    m_robot.GetShooter().ShootTime(5))
                    .Named("DriveForwardShoot")} {}

void DriveForwardShoot::DisabledPeriodic() {
  m_robot.RobotPeriodic();
}

void DriveForwardShoot::Start() {
  frc::commands3::Scheduler::GetDefault().Schedule(m_command);
}

void DriveForwardShoot::Periodic() {
  m_robot.RobotPeriodic();
}

void DriveForwardShoot::End() {
  frc::commands3::Scheduler::GetDefault().Cancel(m_command);
}
