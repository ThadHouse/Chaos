// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/current.h>

#include "../Constants.h"
#include "../hardware/ExpansionHubMotor.h"
#include "../hardware/ExpansionHubServo.h"
#include "Leds.h"

namespace frc {
namespace robot {
namespace subsystems {

/**
 * Shooter subsystem.
 *
 * Controls a single velocity-PID shooter motor (ExpansionHub 1, port 0) and
 * two continuous-rotation feeder servos (ExpansionHub 1, ports 0 and 2).
 * RGB LED feedback indicates velocity error:
 *   - Blue:  Not spinning
 *   - Red:   Error > 2 rot/s
 *   - Green: Error ≤ 2 rot/s (at speed)
 */
class Shooter : public frc2::SubsystemBase {
 public:
  explicit Shooter(Leds& leds);

  /** Periodic update: update LEDs based on velocity error. */
  void Periodic() override;

  /** @return Current shooter wheel velocity in rotations per second. */
  double GetShooterVelocity();

  /** @return Current shooter wheel position in rotations. */
  double GetShooterPosition();

  /** @return Current draw of the shooter motor. */
  units::ampere_t GetShooterCurrent();

  /** @return Whether the expansion hub is connected. */
  bool IsHubConnected();

  /**
   * Sets the shooter target velocity.
   *
   * @param speed Target velocity in rotations per second.
   */
  void SetSpeed(double speed);

  /**
   * Enables or disables the feeder servos.
   * Feeding is suppressed if velocity error exceeds 2 rot/s.
   *
   * @param feed True to enable feeder, false to disable.
   */
  void SetFeed(bool feed);

  /** @return Command that spins shooter at 40 rot/s without feeding. */
  frc2::CommandPtr GetSpinCommand();

  /** @return Command that spins shooter at 40 rot/s and enables feeding. */
  frc2::CommandPtr GetSpinAndFeedCommand();

  /**
   * @return Command that shoots for the given duration then stops.
   *
   * @param time Duration in seconds.
   */
  frc2::CommandPtr ShootTime(double time);

 private:
  frc::robot::hardware::ExpansionHubMotor m_shooterMotor{
      1, ShooterConstants::kShooterMotorPort};
  frc::robot::hardware::ExpansionHubServo m_leftFeederServo{1, 0};
  frc::robot::hardware::ExpansionHubServo m_rightFeederServo{1, 2};

  Leds& m_leds;
  double m_lastSpeed = 0.0;
};

}  // namespace subsystems
}  // namespace robot
}  // namespace frc
