// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "MainTeleop.h"

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/Trigger.h>

using namespace frc::robot::teleopmodes;

MainTeleop::MainTeleop(Robot& robot)
    : m_robot{robot},
      m_joystickDriveCommand{
          m_robot.GetDrive().GetJoystickDriveCommand(m_robot.GetDriverGamepad())} {}

void MainTeleop::DisabledPeriodic() {
  m_robot.RobotPeriodic();
}

void MainTeleop::Start() {
  m_joystickDriveCommand.Schedule();

  auto& shooter = m_robot.GetShooter();
  auto& gamepad = m_robot.GetDriverGamepad();

  frc2::Trigger rightBumper{[&gamepad] { return gamepad.GetRawButton(6); }};
  frc2::Trigger leftBumper{[&gamepad] { return gamepad.GetRawButton(5); }};

  rightBumper.And(leftBumper).WhileTrue(shooter.GetSpinAndFeedCommand());
  rightBumper.And(leftBumper.Negate()).WhileTrue(shooter.GetSpinCommand());
}

void MainTeleop::Periodic() {
  m_robot.RobotPeriodic();
}

void MainTeleop::End() {
  m_joystickDriveCommand.Cancel();
}
