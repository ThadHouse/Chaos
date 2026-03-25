// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/opmode/PeriodicOpMode.hpp>

#include "../Robot.h"

namespace wpi {
namespace robot {
namespace teleopmodes {

/**
 * Main teleop mode.
 *
 * - Left stick XY: field-relative translation
 * - Right stick X: rotation
 * - Right bumper alone: spin shooter at 40 RPM
 * - Right + left bumper: spin and feed
 */
class MainTeleop : public wpi::PeriodicOpMode {
 public:
  explicit MainTeleop(Robot& robot);

  void DisabledPeriodic() override;
  void Start() override;
  void Periodic() override;
  void End() override;

 private:
  Robot& m_robot;
  wpi::cmd::CommandPtr m_joystickDriveCommand;
};

}  // namespace teleopmodes
}  // namespace robot
}  // namespace wpi
