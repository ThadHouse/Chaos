// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "../Robot.h"

namespace frc {
namespace robot {
namespace testopmodes {

/**
 * Test mode: continuously spins the shooter at 40 rot/s.
 *
 * In the C++ port, this logic is scheduled from Robot::TestInit().
 * This class exists as a logical grouping matching the Java @TestOpMode opmode.
 */
class SpinShooter {
 public:
  explicit SpinShooter(Robot& robot);

  void DisabledPeriodic();
  void Start();
  void Periodic();
  void End();

 private:
  Robot& m_robot;
  frc2::CommandPtr m_spinCommand;
};

}  // namespace testopmodes
}  // namespace robot
}  // namespace frc
