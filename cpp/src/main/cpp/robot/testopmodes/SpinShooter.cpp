// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SpinShooter.h"

#include <wpi/commands2/CommandScheduler.hpp>

using namespace wpi::robot::testopmodes;

SpinShooter::SpinShooter(Robot& robot)
    : m_robot{robot},
      m_spinCommand{m_robot.GetShooter().GetSpinCommand()} {}

void SpinShooter::DisabledPeriodic() {
  m_robot.RobotPeriodic();
}

void SpinShooter::Start() {
  m_spinCommand.Schedule();
}

void SpinShooter::Periodic() {
  m_robot.RobotPeriodic();
}

void SpinShooter::End() {
  m_spinCommand.Cancel();
}
