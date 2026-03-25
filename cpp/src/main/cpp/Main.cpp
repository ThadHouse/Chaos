// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Main entry point for the C++ robot program.
 *
 * Do NOT add any static variables to this file, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to change
 * the Robot class template parameter passed to RobotBase::StartRobot.
 */

#include <wpi/framework/RobotBase.hpp>

#include "robot/Robot.h"

int main() {
  return wpi::StartRobot<wpi::robot::Robot>();
}
