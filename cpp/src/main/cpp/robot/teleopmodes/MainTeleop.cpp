// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "MainTeleop.h"

#include <frc/commands3/Scheduler.h>

// Registration macro — equivalent to Java @Teleop annotation
REGISTER_TELEOP(frc::robot::teleopmodes::MainTeleop)

using namespace frc::robot::teleopmodes;

MainTeleop::MainTeleop(Robot& robot)
    : m_robot{robot},
      m_joystickDriveCommand{m_robot.GetDrive().GetJoystickDriveCommand(
          m_robot.GetDriverGamepad().GetHID())} {}

void MainTeleop::DisabledPeriodic() {
  m_robot.RobotPeriodic();
}

void MainTeleop::Start() {
  frc::commands3::Scheduler::GetDefault().Schedule(m_joystickDriveCommand);

  auto& shooter = m_robot.GetShooter();
  auto& driverGamepad = m_robot.GetDriverGamepad();

  driverGamepad.RightBumper()
      .And(driverGamepad.LeftBumper())
      .WhileTrue(shooter.GetSpinAndFeedCommand());

  driverGamepad.RightBumper()
      .And(driverGamepad.LeftBumper().Negate())
      .WhileTrue(shooter.GetSpinCommand());
}

void MainTeleop::Periodic() {
  m_robot.RobotPeriodic();
}

void MainTeleop::End() {
  frc::commands3::Scheduler::GetDefault().Cancel(m_joystickDriveCommand);
}
