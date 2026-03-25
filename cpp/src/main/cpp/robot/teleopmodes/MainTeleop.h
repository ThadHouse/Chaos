// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "../Robot.h"

namespace frc {
namespace robot {
namespace teleopmodes {

/**
 * Main teleop mode.
 *
 * - Left stick XY: field-relative translation
 * - Right stick X: rotation
 * - Right bumper alone: spin shooter at 40 RPM
 * - Right + left bumper: spin and feed
 *
 * In the C++ port, this logic is scheduled from Robot::TeleopInit().
 * This class exists as a logical grouping matching the Java @Teleop opmode.
 */
class MainTeleop {
 public:
  explicit MainTeleop(Robot& robot);

  void DisabledPeriodic();
  void Start();
  void Periodic();
  void End();

 private:
  Robot& m_robot;
  frc2::CommandPtr m_joystickDriveCommand;
};

}  // namespace teleopmodes
}  // namespace robot
}  // namespace frc
