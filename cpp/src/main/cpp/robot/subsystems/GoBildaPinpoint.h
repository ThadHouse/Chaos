// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>
#include <string>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/hardware/bus/I2C.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace frc {
namespace robot {
namespace subsystems {

/**
 * Driver for the goBILDA Pinpoint odometry computer.
 *
 * Communicates over I2C to provide position, heading, and velocity data
 * using dual encoders and an integrated IMU.
 */
class GoBildaPinpoint {
 public:
  // i2c address of the device
  static constexpr uint8_t kDefaultAddress = 0x31;

  // ticks-per-mm for goBILDA odometry pods
  static constexpr float kGoBildaSwingarmPod = 13.26291192f;
  static constexpr float kGoBildar4BarPod = 19.89436789f;

  /** Register map of the I2C device */
  enum class Register : int {
    DEVICE_ID = 1,
    DEVICE_VERSION = 2,
    DEVICE_STATUS = 3,
    DEVICE_CONTROL = 4,
    LOOP_TIME = 5,
    X_ENCODER_VALUE = 6,
    Y_ENCODER_VALUE = 7,
    X_POSITION = 8,
    Y_POSITION = 9,
    H_ORIENTATION = 10,
    X_VELOCITY = 11,
    Y_VELOCITY = 12,
    H_VELOCITY = 13,
    MM_PER_TICK = 14,
    X_POD_OFFSET = 15,
    Y_POD_OFFSET = 16,
    YAW_SCALAR = 17,
    BULK_READ = 18
  };

  /** Device status enum capturing current fault condition */
  enum class DeviceStatus {
    NOT_READY = 0,
    READY = 1,
    CALIBRATING = 1 << 1,
    FAULT_X_POD_NOT_DETECTED = 1 << 2,
    FAULT_Y_POD_NOT_DETECTED = 1 << 3,
    FAULT_NO_PODS_DETECTED = (1 << 2) | (1 << 3),
    FAULT_IMU_RUNAWAY = 1 << 4
  };

  /** Encoder direction configuration */
  enum class EncoderDirection { FORWARD, REVERSED };

  /** Supported goBILDA odometry pod types */
  enum class GoBildaOdometryPods { goBILDA_SWINGARM_POD, goBILDA_4_BAR_POD };

  /** Narrow-scope read options */
  enum class ReadData { ONLY_UPDATE_HEADING };

  explicit GoBildaPinpoint(frc::hardware::bus::I2C::Port port);

  /**
   * Call once per loop to read new data from the Odometry Computer.
   * Data will only update once this is called.
   */
  void Update();

  /**
   * Overload of Update() that reads a narrower set of data for faster reads.
   * Currently only ONLY_UPDATE_HEADING is supported.
   */
  void Update(ReadData data);

  /**
   * Sets the odometry pod positions relative to the robot tracking point.
   *
   * @param xOffset How far sideways (mm) the X (forward) pod is from center.
   *                Left is positive.
   * @param yOffset How far forward (mm) the Y (strafe) pod is from center.
   *                Forward is positive.
   */
  void SetOffsets(units::millimeter_t xOffset, units::millimeter_t yOffset);

  /** Recalibrates the internal IMU. Robot MUST be stationary. */
  void RecalibrateIMU();

  /**
   * Resets position to 0,0,0 and recalibrates the internal IMU.
   * Robot MUST be stationary.
   */
  void ResetPosAndIMU();

  /**
   * Configures encoder directions.
   *
   * @param xEncoder X (forward) pod should increase when moving forward.
   * @param yEncoder Y (strafe) pod should increase when moving left.
   */
  void SetEncoderDirections(EncoderDirection xEncoder,
                            EncoderDirection yEncoder);

  /**
   * Sets encoder resolution using a standard goBILDA pod type.
   *
   * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
   */
  void SetEncoderResolution(GoBildaOdometryPods pods);

  /**
   * Sets encoder resolution in ticks per mm.
   *
   * @param ticksPerMm Ticks per millimeter (typically 10–100).
   */
  void SetEncoderResolution(double ticksPerMm);

  /**
   * Sets the yaw scalar for heading correction.
   *
   * @param yawOffset A scalar applied to the robot's heading.
   */
  void SetYawScalar(double yawOffset);

  /**
   * Overrides the current estimated position.
   *
   * @param pos New robot pose.
   * @return The pose that was set.
   */
  frc::Pose2d SetPosition(const frc::Pose2d& pos);

  /** @return Device ID (should be 1 if functional). */
  int32_t GetDeviceID();

  /** @return Firmware version of the Odometry Computer. */
  int32_t GetDeviceVersion();

  /** @return Current yaw scalar. */
  float GetYawScalar();

  /** @return Current device status/fault state. */
  DeviceStatus GetDeviceStatus();

  /** @return Most recent loop time in microseconds. */
  int32_t GetLoopTime();

  /** @return Pinpoint loop frequency in Hz. */
  double GetFrequency();

  /** @return Raw X (forward) encoder value in ticks. */
  int32_t GetEncoderX();

  /** @return Raw Y (strafe) encoder value in ticks. */
  int32_t GetEncoderY();

  /** @return Estimated X (forward) position. */
  units::millimeter_t GetPosX();

  /** @return Estimated Y (strafe) position. */
  units::millimeter_t GetPosY();

  /** @return Estimated heading. */
  frc::Rotation2d GetHeading();

  /** @return Estimated X velocity. */
  units::meters_per_second_t GetVelX();

  /** @return Estimated Y velocity. */
  units::meters_per_second_t GetVelY();

  /** @return Estimated heading angular velocity. */
  units::radians_per_second_t GetHeadingVelocity();

  /** @return X pod offset (separate I2C read — avoid calling every loop). */
  units::millimeter_t GetXOffset();

  /** @return Y pod offset (separate I2C read — avoid calling every loop). */
  units::millimeter_t GetYOffset();

  /** @return Current estimated pose. */
  frc::Pose2d GetPosition();

 private:
  frc::hardware::bus::I2C m_i2c;

  int32_t m_deviceStatus = 0;
  int32_t m_loopTime = 0;
  int32_t m_xEncoderValue = 0;
  int32_t m_yEncoderValue = 0;
  float m_xPosition = 0.0f;
  float m_yPosition = 0.0f;
  float m_hOrientation = 0.0f;
  float m_xVelocity = 0.0f;
  float m_yVelocity = 0.0f;
  float m_hVelocity = 0.0f;

  void WriteInt(Register reg, int32_t value);
  void WriteFloat(Register reg, float value);
  int32_t ReadInt(Register reg);
  float ReadFloat(Register reg);
  DeviceStatus LookupStatus(int32_t s);

  // Helper to read a little-endian int32 from a byte buffer
  static int32_t ReadInt32LE(const uint8_t* buf, int offset);
  // Helper to read a little-endian float from a byte buffer
  static float ReadFloatLE(const uint8_t* buf, int offset);
};

}  // namespace subsystems
}  // namespace robot
}  // namespace frc
