// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DoNothingAuto.h"

using namespace wpi::robot::autoopmodes;

DoNothingAuto::DoNothingAuto(Robot& robot) : m_robot{robot} {}

void DoNothingAuto::DisabledPeriodic() {
  m_robot.RobotPeriodic();
}

void DoNothingAuto::Periodic() {
  m_robot.RobotPeriodic();
}
