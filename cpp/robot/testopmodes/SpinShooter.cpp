// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SpinShooter.h"

#include <frc/commands3/Scheduler.h>

// Registration macro — equivalent to Java @TestOpMode annotation
REGISTER_TEST_OP_MODE(frc::robot::testopmodes::SpinShooter)

using namespace frc::robot::testopmodes;

SpinShooter::SpinShooter(Robot& robot)
    : m_robot{robot},
      m_spinCommand{m_robot.GetShooter().GetSpinCommand()} {}

void SpinShooter::DisabledPeriodic() {
  m_robot.RobotPeriodic();
}

void SpinShooter::Start() {
  frc::commands3::Scheduler::GetDefault().Schedule(m_spinCommand);
}

void SpinShooter::Periodic() {
  m_robot.RobotPeriodic();
}

void SpinShooter::End() {
  frc::commands3::Scheduler::GetDefault().Cancel(m_spinCommand);
}
