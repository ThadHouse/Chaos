// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/commands3/Scheduler.h>
#include <frc/system/DataLogManager.h>

#include "Constants.h"

using namespace frc::robot;

Robot::Robot() {
  // First call starts logging to a custom path on the SystemCore target;
  // second call enables default logging. Both mirror the Java implementation.
  frc::DataLogManager::Start("/home/systemcore/logs");
  frc::DataLogManager::Start();
}

void Robot::RobotPeriodic() {
  frc::commands3::Scheduler::GetDefault().Run();
}

units::ampere_t Robot::GetRobotCurrent() {
  double voltage = m_currentReading.GetVoltage();
  double current = (voltage / 3.3) * 50.0;
  return units::ampere_t{current};
}

void Robot::NonePeriodic() {
  RobotPeriodic();
}
