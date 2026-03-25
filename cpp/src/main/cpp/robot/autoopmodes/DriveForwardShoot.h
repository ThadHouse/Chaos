// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "../Robot.h"

namespace frc {
namespace robot {
namespace autoopmodes {

/**
 * Autonomous mode: drive forward for 2 seconds, then shoot for 5 seconds.
 *
 * In the C++ port, this logic is scheduled from Robot::AutonomousInit().
 * This class exists as a logical grouping matching the Java @Autonomous opmode.
 */
class DriveForwardShoot {
 public:
  explicit DriveForwardShoot(Robot& robot);

  void DisabledPeriodic();
  void Start();
  void Periodic();
  void End();

 private:
  Robot& m_robot;
  frc2::CommandPtr m_command;
};

}  // namespace autoopmodes
}  // namespace robot
}  // namespace frc
