// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/commands3/Command.h>
#include <frc/opmode/PeriodicOpMode.h>

#include "../Robot.h"

namespace frc {
namespace robot {
namespace teleopmodes {

/**
 * Main teleop OpMode.
 *
 * - Left stick XY: field-relative translation
 * - Right stick X: rotation
 * - Right bumper alone: spin shooter at 40 RPM
 * - Right + left bumper: spin and feed
 *
 * Registered with the @Teleop annotation equivalent:
 *   REGISTER_TELEOP(MainTeleop)
 */
class MainTeleop : public frc::opmode::PeriodicOpMode {
 public:
  explicit MainTeleop(Robot& robot);

  void DisabledPeriodic() override;
  void Start() override;
  void Periodic() override;
  void End() override;

 private:
  Robot& m_robot;
  frc::commands3::Command m_joystickDriveCommand;
};

}  // namespace teleopmodes
}  // namespace robot
}  // namespace frc
