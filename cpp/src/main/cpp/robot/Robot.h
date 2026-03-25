// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogInput.h>
#include <frc/GenericHID.h>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
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
class Robot : public frc::TimedRobot {
 public:
  Robot();

  void RobotInit() override;

  /** Called every robot loop iteration regardless of mode. */
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  /** @return Reference to the drive subsystem. */
  subsystems::DriveSubsystemNew& GetDrive() { return m_robotDrive; }

  /** @return Reference to the shooter subsystem. */
  subsystems::Shooter& GetShooter() { return m_shooter; }

  /** @return Reference to the LED subsystem. */
  subsystems::Leds& GetLeds() { return m_leds; }

  /** @return Reference to the driver gamepad. */
  frc::GenericHID& GetDriverGamepad() { return m_driverController; }

  /** @return Estimated robot current draw in amps. */
  units::ampere_t GetRobotCurrent();

 private:
  subsystems::Leds m_leds;
  subsystems::Shooter m_shooter{m_leds};
  subsystems::DriveSubsystemNew m_robotDrive;

  frc::AnalogInput m_currentReading{5};

  frc::GenericHID m_driverController{OIConstants::kDriverControllerPort};

  frc2::CommandPtr m_autonomousCommand{frc2::cmd::None()};
  frc2::CommandPtr m_teleopCommand{frc2::cmd::None()};
  frc2::CommandPtr m_testCommand{frc2::cmd::None()};
};

}  // namespace robot
}  // namespace frc
