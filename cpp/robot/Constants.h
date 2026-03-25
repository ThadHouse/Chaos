// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/hardware/bus/I2C.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

/**
 * The Constants namespace provides a convenient place for teams to hold
 * robot-wide numerical or boolean constants. All constants should be declared
 * as constexpr. Do not put anything functional in this namespace.
 */
namespace DriveConstants {
inline constexpr units::meters_per_second_t kMaxSpeed{0.6};
inline constexpr units::radians_per_second_t kMaxAngularSpeed{1.0};
inline constexpr units::meters_per_second_t kMaxWheelSpeed{1.0};

inline constexpr int kFrontLeftMotorPort = 1;
inline constexpr int kRearLeftMotorPort = 0;
inline constexpr int kFrontRightMotorPort = 2;
inline constexpr int kRearRightMotorPort = 3;

inline constexpr bool kFrontLeftEncoderReversed = false;
inline constexpr bool kRearLeftEncoderReversed = false;
inline constexpr bool kFrontRightEncoderReversed = true;
inline constexpr bool kRearRightEncoderReversed = true;

inline constexpr units::millimeter_t kTrackWidth{416.0};
inline constexpr units::millimeter_t kTrackWidthToCenter{kTrackWidth / 2.0};
// Distance between centers of right and left wheels on robot
inline constexpr units::millimeter_t kWheelBase{336.0};  // 14 large holes * 24mm per large hole
inline constexpr units::millimeter_t kWheelBaseToCenter{kWheelBase / 2.0};
// Distance between centers of front and back wheels on robot

// Created inline (not constexpr) because MecanumDriveKinematics is not trivial
inline const frc::MecanumDriveKinematics kDriveKinematics{
    frc::Translation2d{kWheelBaseToCenter, kTrackWidthToCenter},
    frc::Translation2d{kWheelBaseToCenter, -kTrackWidthToCenter},
    frc::Translation2d{-kWheelBaseToCenter, kTrackWidthToCenter},
    frc::Translation2d{-kWheelBaseToCenter, -kTrackWidthToCenter}};

inline constexpr double kEncoderCPR = 537.7;
inline constexpr units::meter_t kWheelDiameterMeters{0.15};
inline constexpr double kEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (0.15 * std::numbers::pi) / kEncoderCPR;

inline constexpr double kS = 0.43007;
inline constexpr double kV = 4.9336;
inline constexpr double kA = 0.71773;

inline constexpr double kP = 2.7238;

inline constexpr units::millimeter_t kYOffset{120.0};  // 15 holes * 8mm per hole
inline constexpr units::millimeter_t kXOffset{92.0};   // 11.5 holes * 8mm per hole
}  // namespace DriveConstants

namespace OIConstants {
inline constexpr int kDriverControllerPort = 0;
inline constexpr double kDriveDeadband = 0.05;
}  // namespace OIConstants

namespace ShooterConstants {
inline constexpr int kShooterMotorPort = 0;
inline constexpr int kEncoderPort = 0;
inline constexpr frc::hardware::bus::I2C::Port kI2cPort =
    frc::hardware::bus::I2C::Port::kPort0;

inline constexpr double kP = 0.16666;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kS = 0.56532;
inline constexpr double kA = 0.17586;
inline constexpr double kV = 0.13448;

inline constexpr int kEncoderCPR = 28;
inline constexpr double kEncoderDistancePerPulse =
    1.0 / kEncoderCPR;  // Distance units in rotations
}  // namespace ShooterConstants

namespace AutoConstants {
inline constexpr units::meters_per_second_t kMaxSpeedMetersPerSecond{3.0};
inline constexpr units::meters_per_second_squared_t
    kMaxAccelerationMetersPerSecondSquared{3.0};
inline constexpr units::radians_per_second_t kMaxAngularSpeedRadiansPerSecond{
    std::numbers::pi};
inline constexpr units::radians_per_second_squared_t
    kMaxAngularSpeedRadiansPerSecondSquared{std::numbers::pi};

inline constexpr double kPXController = 0.5;
inline constexpr double kPYController = 0.5;
inline constexpr double kPThetaController = 0.5;
}  // namespace AutoConstants
