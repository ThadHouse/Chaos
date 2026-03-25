// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/opmode/PeriodicOpMode.h>

#include "../Robot.h"

namespace frc {
namespace robot {
namespace autoopmodes {

/**
 * Autonomous OpMode that does nothing except run the robot periodic loop.
 *
 * Registered with the @Autonomous annotation equivalent:
 *   REGISTER_AUTONOMOUS(DoNothingAuto)
 */
class DoNothingAuto : public frc::opmode::PeriodicOpMode {
 public:
  explicit DoNothingAuto(Robot& robot);

  void DisabledPeriodic() override;
  void Periodic() override;

 private:
  Robot& m_robot;
};

}  // namespace autoopmodes
}  // namespace robot
}  // namespace frc
