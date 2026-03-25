// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/opmode/PeriodicOpMode.hpp>

#include "../Robot.h"

namespace wpi {
namespace robot {
namespace testopmodes {

/**
 * Test mode: continuously spins the shooter at 40 rot/s.
 */
class SpinShooter : public wpi::PeriodicOpMode {
 public:
  explicit SpinShooter(Robot& robot);

  void DisabledPeriodic() override;
  void Start() override;
  void Periodic() override;
  void End() override;

 private:
  Robot& m_robot;
  wpi::cmd::CommandPtr m_spinCommand;
};

}  // namespace testopmodes
}  // namespace robot
}  // namespace wpi
