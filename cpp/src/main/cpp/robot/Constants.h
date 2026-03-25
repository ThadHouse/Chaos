// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <wpi/hardware/bus/I2C.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/MecanumDriveKinematics.hpp>
#include <wpi/units/acceleration.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

/**
 * The Constants namespace provides a convenient place for teams to hold
 * robot-wide numerical or boolean constants. All constants should be declared
 * as constexpr. Do not put anything functional in this namespace.
 */
namespace DriveConstants {
inline constexpr wpi::units::meters_per_second_t kMaxSpeed{0.6};
inline constexpr wpi::units::radians_per_second_t kMaxAngularSpeed{1.0};
inline constexpr wpi::units::meters_per_second_t kMaxWheelSpeed{1.0};

inline constexpr int kFrontLeftMotorPort = 1;
inline constexpr int kRearLeftMotorPort = 0;
inline constexpr int kFrontRightMotorPort = 2;
inline constexpr int kRearRightMotorPort = 3;

inline constexpr bool kFrontLeftEncoderReversed = false;
inline constexpr bool kRearLeftEncoderReversed = false;
inline constexpr bool kFrontRightEncoderReversed = true;
inline constexpr bool kRearRightEncoderReversed = true;

inline constexpr wpi::units::millimeter_t kTrackWidth{416.0};
inline constexpr wpi::units::millimeter_t kTrackWidthToCenter{kTrackWidth / 2.0};
// Distance between centers of right and left wheels on robot
inline constexpr wpi::units::millimeter_t kWheelBase{336.0};  // 14 large holes * 24mm per large hole
inline constexpr wpi::units::millimeter_t kWheelBaseToCenter{kWheelBase / 2.0};
// Distance between centers of front and back wheels on robot

// Created inline (not constexpr) because MecanumDriveKinematics is not trivial
inline const wpi::math::MecanumDriveKinematics kDriveKinematics{
    wpi::math::Translation2d{kWheelBaseToCenter, kTrackWidthToCenter},
    wpi::math::Translation2d{kWheelBaseToCenter, -kTrackWidthToCenter},
    wpi::math::Translation2d{-kWheelBaseToCenter, kTrackWidthToCenter},
    wpi::math::Translation2d{-kWheelBaseToCenter, -kTrackWidthToCenter}};

inline constexpr double kEncoderCPR = 537.7;
inline constexpr wpi::units::meter_t kWheelDiameterMeters{0.15};
inline constexpr double kEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (0.15 * std::numbers::pi) / kEncoderCPR;

inline constexpr double kS = 0.43007;
inline constexpr double kV = 4.9336;
inline constexpr double kA = 0.71773;

inline constexpr double kP = 2.7238;

inline constexpr wpi::units::millimeter_t kYOffset{120.0};  // 15 holes * 8mm per hole
inline constexpr wpi::units::millimeter_t kXOffset{92.0};   // 11.5 holes * 8mm per hole
}  // namespace DriveConstants

namespace OIConstants {
inline constexpr int kDriverControllerPort = 0;
inline constexpr double kDriveDeadband = 0.05;
}  // namespace OIConstants

namespace ShooterConstants {
inline constexpr int kShooterMotorPort = 0;
inline constexpr int kEncoderPort = 0;
inline constexpr wpi::I2C::Port kI2cPort = wpi::I2C::Port::kPort0;

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
inline constexpr wpi::units::meters_per_second_t kMaxSpeedMetersPerSecond{3.0};
inline constexpr wpi::units::meters_per_second_squared_t
    kMaxAccelerationMetersPerSecondSquared{3.0};
inline constexpr wpi::units::radians_per_second_t kMaxAngularSpeedRadiansPerSecond{
    std::numbers::pi};
inline constexpr wpi::units::radians_per_second_squared_t
    kMaxAngularSpeedRadiansPerSecondSquared{std::numbers::pi};

inline constexpr double kPXController = 0.5;
inline constexpr double kPYController = 0.5;
inline constexpr double kPThetaController = 0.5;
}  // namespace AutoConstants
