// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DoNothingAuto.h"

// Registration macro — equivalent to Java @Autonomous annotation
REGISTER_AUTONOMOUS(frc::robot::autoopmodes::DoNothingAuto)

using namespace frc::robot::autoopmodes;

DoNothingAuto::DoNothingAuto(Robot& robot) : m_robot{robot} {}

void DoNothingAuto::DisabledPeriodic() {
  m_robot.RobotPeriodic();
}

void DoNothingAuto::Periodic() {
  m_robot.RobotPeriodic();
}
