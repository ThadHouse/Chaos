// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/commands3/button/CommandGamepad.h>
#include <frc/framework/OpModeRobot.h>
#include <frc/hardware/discrete/AnalogInput.h>
#include <units/current.h>

#include "Constants.h"
#include "subsystems/DriveSubsystemNew.h"
#include "subsystems/Leds.h"
#include "subsystems/Shooter.h"

namespace frc {
namespace robot {

/**
 * Main robot class.
 *
 * Owns the three subsystems (drive, shooter, LEDs) and driver gamepad.
 * Manages periodic telemetry logging and system-wide initialization.
 */
class Robot : public frc::OpModeRobot {
 public:
  Robot();

  /** Called every robot loop iteration. Runs the scheduler and logs data. */
  void RobotPeriodic();

  /** @return Reference to the drive subsystem. */
  subsystems::DriveSubsystemNew& GetDrive() { return m_robotDrive; }

  /** @return Reference to the shooter subsystem. */
  subsystems::Shooter& GetShooter() { return m_shooter; }

  /** @return Reference to the LED subsystem. */
  subsystems::Leds& GetLeds() { return m_leds; }

  /** @return Reference to the driver gamepad. */
  frc::commands3::button::CommandGamepad& GetDriverGamepad() {
    return m_driverController;
  }

  /** @return Estimated robot current draw in amps. */
  units::ampere_t GetRobotCurrent();

  void NonePeriodic() override;

 private:
  subsystems::Leds m_leds;
  subsystems::Shooter m_shooter{m_leds};
  subsystems::DriveSubsystemNew m_robotDrive;

  frc::hardware::discrete::AnalogInput m_currentReading{5};

  frc::commands3::button::CommandGamepad m_driverController{
      OIConstants::kDriverControllerPort};
};

}  // namespace robot
}  // namespace frc
