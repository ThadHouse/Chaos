// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <units/time.h>

#include "Constants.h"

using namespace frc::robot;

Robot::Robot() {
  // Mirror the Java implementation: start logging first to a custom path,
  // then with defaults.
  frc::DataLogManager::Start("/home/systemcore/logs");
  frc::DataLogManager::Start();
}

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
  // Cancel any previous command and rebuild
  m_autonomousCommand.Cancel();

  // Drive forward for 2 s then shoot for 5 s (mirrors DriveForwardShoot auto)
  m_autonomousCommand =
      m_robotDrive.DriveForwardTime(2.0)
          .AndThen(m_shooter.ShootTime(5.0));
  m_autonomousCommand.Schedule();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_autonomousCommand.Cancel();

  // Joystick drive: left stick XY = translation, right stick X = rotation
  m_teleopCommand = m_robotDrive.GetJoystickDriveCommand(m_driverController);
  m_teleopCommand.Schedule();

  // Right bumper (button 6) → spin; right + left bumper (buttons 5+6) → shoot
  frc2::Trigger rightBumper{[this] {
    return m_driverController.GetRawButton(6);
  }};
  frc2::Trigger leftBumper{[this] {
    return m_driverController.GetRawButton(5);
  }};

  rightBumper.And(leftBumper.Negate())
      .WhileTrue(m_shooter.GetSpinCommand());
  rightBumper.And(leftBumper)
      .WhileTrue(m_shooter.GetSpinAndFeedCommand());
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  // SpinShooter test: continuously spin at 40 rot/s
  m_testCommand = m_shooter.GetSpinCommand();
  m_testCommand.Schedule();
}

void Robot::TestPeriodic() {}

units::ampere_t Robot::GetRobotCurrent() {
  double voltage = m_currentReading.GetVoltage();
  double current = (voltage / 3.3) * 50.0;
  return units::ampere_t{current};
}
