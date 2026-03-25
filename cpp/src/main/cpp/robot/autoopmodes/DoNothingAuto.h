// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "../Robot.h"

namespace frc {
namespace robot {
namespace autoopmodes {

/**
 * Autonomous mode that does nothing except run the robot periodic loop.
 *
 * In the C++ port the robot framework is frc::TimedRobot, so mode selection
 * is handled by Robot::AutonomousInit / AutonomousPeriodic. This class exists
 * as a logical grouping for the "do nothing" auto logic.
 */
class DoNothingAuto {
 public:
  explicit DoNothingAuto(Robot& robot);

  void DisabledPeriodic();
  void Periodic();

 private:
  Robot& m_robot;
};

}  // namespace autoopmodes
}  // namespace robot
}  // namespace frc
