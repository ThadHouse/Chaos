// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/opmode/PeriodicOpMode.hpp>

#include "../Robot.h"

namespace wpi {
namespace robot {
namespace autoopmodes {

/**
 * Autonomous mode that does nothing except run the robot periodic loop.
 */
class DoNothingAuto : public wpi::PeriodicOpMode {
 public:
  explicit DoNothingAuto(Robot& robot);

  void DisabledPeriodic() override;
  void Periodic() override;

 private:
  Robot& m_robot;
};

}  // namespace autoopmodes
}  // namespace robot
}  // namespace wpi
