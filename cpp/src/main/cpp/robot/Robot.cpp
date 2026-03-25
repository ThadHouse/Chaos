// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/system/DataLogManager.hpp>

#include "autoopmodes/DoNothingAuto.h"
#include "autoopmodes/DriveForwardShoot.h"
#include "teleopmodes/MainTeleop.h"
#include "testopmodes/SpinShooter.h"

using namespace wpi::robot;

Robot::Robot() {
  wpi::DataLogManager::Start("/home/systemcore/logs");
  wpi::DataLogManager::Start();

  AddOpMode<autoopmodes::DoNothingAuto>(wpi::RobotMode::AUTONOMOUS, "DoNothingAuto");
  AddOpMode<autoopmodes::DriveForwardShoot>(wpi::RobotMode::AUTONOMOUS, "DriveForwardShoot");
  AddOpMode<teleopmodes::MainTeleop>(wpi::RobotMode::TELEOPERATED, "MainTeleop");
  AddOpMode<testopmodes::SpinShooter>(wpi::RobotMode::TEST, "SpinShooter");
}

void Robot::RobotPeriodic() {
  wpi::cmd::CommandScheduler::GetInstance().Run();
}

void Robot::NonePeriodic() {
  RobotPeriodic();
}

wpi::units::ampere_t Robot::GetRobotCurrent() {
  double voltage = m_currentReading.GetVoltage();
  double current = (voltage / 3.3) * 50.0;
  return wpi::units::ampere_t{current};
}
