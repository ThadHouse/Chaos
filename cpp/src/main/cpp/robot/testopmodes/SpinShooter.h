// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/commands3/Command.h>
#include <frc/opmode/PeriodicOpMode.h>

#include "../Robot.h"

namespace frc {
namespace robot {
namespace testopmodes {

/**
 * Test OpMode: continuously spins the shooter at 40 rot/s.
 *
 * Registered with the @TestOpMode annotation equivalent:
 *   REGISTER_TEST_OP_MODE(SpinShooter)
 */
class SpinShooter : public frc::opmode::PeriodicOpMode {
 public:
  explicit SpinShooter(Robot& robot);

  void DisabledPeriodic() override;
  void Start() override;
  void Periodic() override;
  void End() override;

 private:
  Robot& m_robot;
  frc::commands3::Command m_spinCommand;
};

}  // namespace testopmodes
}  // namespace robot
}  // namespace frc
