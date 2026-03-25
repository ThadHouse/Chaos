// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/opmode/PeriodicOpMode.hpp>

#include "../Robot.h"

namespace wpi {
namespace robot {
namespace autoopmodes {

/**
 * Autonomous mode: drive forward for 2 seconds, then shoot for 5 seconds.
 */
class DriveForwardShoot : public wpi::PeriodicOpMode {
 public:
  explicit DriveForwardShoot(Robot& robot);

  void DisabledPeriodic() override;
  void Start() override;
  void Periodic() override;
  void End() override;

 private:
  Robot& m_robot;
  wpi::cmd::CommandPtr m_command;
};

}  // namespace autoopmodes
}  // namespace robot
}  // namespace wpi
