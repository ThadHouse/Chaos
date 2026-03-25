// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/driverstation/GenericHID.hpp>
#include <wpi/framework/OpModeRobot.hpp>
#include <wpi/hardware/discrete/AnalogInput.hpp>
#include <wpi/units/current.hpp>

#include "Constants.h"
#include "subsystems/DriveSubsystemNew.h"
#include "subsystems/Leds.h"
#include "subsystems/Shooter.h"

namespace wpi {
namespace robot {

/**
 * Main robot class.
 *
 * Owns the three subsystems (drive, shooter, LEDs) and driver gamepad.
 * Manages periodic telemetry logging and system-wide initialization.
 */
class Robot : public wpi::OpModeRobot<Robot> {
 public:
  Robot();

  /** Called every robot loop to run the command scheduler. */
  void RobotPeriodic();

  /** Called when no opmode is selected. */
  void NonePeriodic() override;

  /** @return Reference to the drive subsystem. */
  subsystems::DriveSubsystemNew& GetDrive() { return m_robotDrive; }

  /** @return Reference to the shooter subsystem. */
  subsystems::Shooter& GetShooter() { return m_shooter; }

  /** @return Reference to the LED subsystem. */
  subsystems::Leds& GetLeds() { return m_leds; }

  /** @return Reference to the driver gamepad. */
  wpi::GenericHID& GetDriverGamepad() { return m_driverController; }

  /** @return Estimated robot current draw in amps. */
  wpi::units::ampere_t GetRobotCurrent();

 private:
  subsystems::Leds m_leds;
  subsystems::Shooter m_shooter{m_leds};
  subsystems::DriveSubsystemNew m_robotDrive;

  wpi::AnalogInput m_currentReading{5};

  wpi::GenericHID m_driverController{OIConstants::kDriverControllerPort};
};

}  // namespace robot
}  // namespace wpi
