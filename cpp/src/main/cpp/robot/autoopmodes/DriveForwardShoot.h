// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/commands3/Command.h>
#include <frc/opmode/PeriodicOpMode.h>

#include "../Robot.h"

namespace frc {
namespace robot {
namespace autoopmodes {

/**
 * Autonomous OpMode: drive forward for 2 seconds, then shoot for 5 seconds.
 *
 * Registered with the @Autonomous annotation equivalent:
 *   REGISTER_AUTONOMOUS(DriveForwardShoot)
 */
class DriveForwardShoot : public frc::opmode::PeriodicOpMode {
 public:
  explicit DriveForwardShoot(Robot& robot);

  void DisabledPeriodic() override;
  void Start() override;
  void Periodic() override;
  void End() override;

 private:
  Robot& m_robot;
  frc::commands3::Command m_command;
};

}  // namespace autoopmodes
}  // namespace robot
}  // namespace frc
