// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "GoBildaPinpoint.h"

#include <cstring>
#include <iostream>

using namespace wpi::robot::subsystems;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

GoBildaPinpoint::GoBildaPinpoint(wpi::I2C::Port port)
    : m_i2c{port, kDefaultAddress} {}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

int32_t GoBildaPinpoint::ReadInt32LE(const uint8_t* buf, int offset) {
  int32_t value = 0;
  std::memcpy(&value, buf + offset, sizeof(value));
  return value;
}

float GoBildaPinpoint::ReadFloatLE(const uint8_t* buf, int offset) {
  float value = 0.0f;
  std::memcpy(&value, buf + offset, sizeof(value));
  return value;
}

void GoBildaPinpoint::WriteInt(Register reg, int32_t i) {
  uint8_t buffer[5];
  buffer[0] = static_cast<uint8_t>(reg);
  // Write as little-endian
  buffer[1] = static_cast<uint8_t>(i & 0xFF);
  buffer[2] = static_cast<uint8_t>((i >> 8) & 0xFF);
  buffer[3] = static_cast<uint8_t>((i >> 16) & 0xFF);
  buffer[4] = static_cast<uint8_t>((i >> 24) & 0xFF);

  if (m_i2c.WriteBulk(buffer, 5)) {
    std::cerr << "GoBildaPinpoint: Failed to write int register" << "\n";
  }
}

void GoBildaPinpoint::WriteFloat(Register reg, float f) {
  uint8_t buffer[5];
  buffer[0] = static_cast<uint8_t>(reg);
  // Copy float bytes as little-endian
  std::memcpy(buffer + 1, &f, sizeof(f));

  if (m_i2c.WriteBulk(buffer, 5)) {
    std::cerr << "GoBildaPinpoint: Failed to write float register" << "\n";
  }
}

int32_t GoBildaPinpoint::ReadInt(Register reg) {
  uint8_t buffer[4];
  if (m_i2c.Read(static_cast<int>(reg), 4, buffer)) {
    std::cerr << "GoBildaPinpoint: Failed to read int register" << "\n";
    return 0;
  }
  return ReadInt32LE(buffer, 0);
}

float GoBildaPinpoint::ReadFloat(Register reg) {
  uint8_t buffer[4];
  if (m_i2c.Read(static_cast<int>(reg), 4, buffer)) {
    std::cerr << "GoBildaPinpoint: Failed to read float register" << "\n";
    return 0.0f;
  }
  return ReadFloatLE(buffer, 0);
}

GoBildaPinpoint::DeviceStatus GoBildaPinpoint::LookupStatus(int32_t s) {
  if ((s & static_cast<int>(DeviceStatus::CALIBRATING)) != 0) {
    return DeviceStatus::CALIBRATING;
  }
  bool xPodDetected =
      (s & static_cast<int>(DeviceStatus::FAULT_X_POD_NOT_DETECTED)) == 0;
  bool yPodDetected =
      (s & static_cast<int>(DeviceStatus::FAULT_Y_POD_NOT_DETECTED)) == 0;

  if (!xPodDetected && !yPodDetected) {
    return DeviceStatus::FAULT_NO_PODS_DETECTED;
  }
  if (!xPodDetected) {
    return DeviceStatus::FAULT_X_POD_NOT_DETECTED;
  }
  if (!yPodDetected) {
    return DeviceStatus::FAULT_Y_POD_NOT_DETECTED;
  }
  if ((s & static_cast<int>(DeviceStatus::FAULT_IMU_RUNAWAY)) != 0) {
    return DeviceStatus::FAULT_IMU_RUNAWAY;
  }
  if ((s & static_cast<int>(DeviceStatus::READY)) != 0) {
    return DeviceStatus::READY;
  }
  return DeviceStatus::NOT_READY;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void GoBildaPinpoint::Update() {
  uint8_t buffer[40];
  if (m_i2c.Read(static_cast<int>(Register::BULK_READ), 40, buffer)) {
    // Silently ignore read errors (same as Java implementation)
    return;
  }
  m_deviceStatus = ReadInt32LE(buffer, 0);
  m_loopTime = ReadInt32LE(buffer, 4);
  m_xEncoderValue = ReadInt32LE(buffer, 8);
  m_yEncoderValue = ReadInt32LE(buffer, 12);
  m_xPosition = ReadFloatLE(buffer, 16);
  m_yPosition = ReadFloatLE(buffer, 20);
  m_hOrientation = ReadFloatLE(buffer, 24);
  m_xVelocity = ReadFloatLE(buffer, 28);
  m_yVelocity = ReadFloatLE(buffer, 32);
  m_hVelocity = ReadFloatLE(buffer, 36);
}

void GoBildaPinpoint::Update(ReadData data) {
  if (data == ReadData::ONLY_UPDATE_HEADING) {
    uint8_t buffer[4];
    if (m_i2c.Read(static_cast<int>(Register::H_ORIENTATION), 4, buffer)) {
      std::cerr << "GoBildaPinpoint: Failed to read heading register" << "\n";
      return;
    }
    m_hOrientation = ReadFloatLE(buffer, 0);
  }
}

void GoBildaPinpoint::SetOffsets(wpi::units::millimeter_t xOffset,
                                  wpi::units::millimeter_t yOffset) {
  WriteFloat(Register::X_POD_OFFSET, static_cast<float>(xOffset.value()));
  WriteFloat(Register::Y_POD_OFFSET, static_cast<float>(yOffset.value()));
}

void GoBildaPinpoint::RecalibrateIMU() {
  WriteInt(Register::DEVICE_CONTROL, 1 << 0);
}

void GoBildaPinpoint::ResetPosAndIMU() {
  WriteInt(Register::DEVICE_CONTROL, 1 << 1);
}

void GoBildaPinpoint::SetEncoderDirections(EncoderDirection xEncoder,
                                            EncoderDirection yEncoder) {
  if (xEncoder == EncoderDirection::FORWARD) {
    WriteInt(Register::DEVICE_CONTROL, 1 << 5);
  }
  if (xEncoder == EncoderDirection::REVERSED) {
    WriteInt(Register::DEVICE_CONTROL, 1 << 4);
  }
  if (yEncoder == EncoderDirection::FORWARD) {
    WriteInt(Register::DEVICE_CONTROL, 1 << 3);
  }
  if (yEncoder == EncoderDirection::REVERSED) {
    WriteInt(Register::DEVICE_CONTROL, 1 << 2);
  }
}

void GoBildaPinpoint::SetEncoderResolution(GoBildaOdometryPods pods) {
  if (pods == GoBildaOdometryPods::goBILDA_SWINGARM_POD) {
    WriteFloat(Register::MM_PER_TICK, kGoBildaSwingarmPod);
  }
  if (pods == GoBildaOdometryPods::goBILDA_4_BAR_POD) {
    WriteFloat(Register::MM_PER_TICK, kGoBildar4BarPod);
  }
}

void GoBildaPinpoint::SetEncoderResolution(double ticksPerMm) {
  WriteFloat(Register::MM_PER_TICK, static_cast<float>(ticksPerMm));
}

void GoBildaPinpoint::SetYawScalar(double yawOffset) {
  WriteFloat(Register::YAW_SCALAR, static_cast<float>(yawOffset));
}

wpi::math::Pose2d GoBildaPinpoint::SetPosition(const wpi::math::Pose2d& pos) {
  WriteFloat(Register::X_POSITION,
             static_cast<float>(wpi::units::millimeter_t{pos.X()}.value()));
  WriteFloat(Register::Y_POSITION,
             static_cast<float>(wpi::units::millimeter_t{pos.Y()}.value()));
  WriteFloat(Register::H_ORIENTATION,
             static_cast<float>(pos.Rotation().Radians().value()));
  return pos;
}

int32_t GoBildaPinpoint::GetDeviceID() {
  return ReadInt(Register::DEVICE_ID);
}

int32_t GoBildaPinpoint::GetDeviceVersion() {
  return ReadInt(Register::DEVICE_VERSION);
}

float GoBildaPinpoint::GetYawScalar() {
  return ReadFloat(Register::YAW_SCALAR);
}

GoBildaPinpoint::DeviceStatus GoBildaPinpoint::GetDeviceStatus() {
  return LookupStatus(m_deviceStatus);
}

int32_t GoBildaPinpoint::GetLoopTime() {
  return m_loopTime;
}

double GoBildaPinpoint::GetFrequency() {
  if (m_loopTime != 0) {
    return 1000000.0 / m_loopTime;
  }
  return 0.0;
}

int32_t GoBildaPinpoint::GetEncoderX() {
  return m_xEncoderValue;
}

int32_t GoBildaPinpoint::GetEncoderY() {
  return m_yEncoderValue;
}

wpi::units::millimeter_t GoBildaPinpoint::GetPosX() {
  return wpi::units::millimeter_t{m_xPosition};
}

wpi::units::millimeter_t GoBildaPinpoint::GetPosY() {
  return wpi::units::millimeter_t{m_yPosition};
}

wpi::math::Rotation2d GoBildaPinpoint::GetHeading() {
  return wpi::math::Rotation2d{wpi::units::radian_t{m_hOrientation}};
}

wpi::units::meters_per_second_t GoBildaPinpoint::GetVelX() {
  return wpi::units::meters_per_second_t{m_xVelocity / 1000.0};
}

wpi::units::meters_per_second_t GoBildaPinpoint::GetVelY() {
  return wpi::units::meters_per_second_t{m_yVelocity / 1000.0};
}

wpi::units::radians_per_second_t GoBildaPinpoint::GetHeadingVelocity() {
  return wpi::units::radians_per_second_t{m_hVelocity};
}

wpi::units::millimeter_t GoBildaPinpoint::GetXOffset() {
  return wpi::units::millimeter_t{ReadFloat(Register::X_POD_OFFSET)};
}

wpi::units::millimeter_t GoBildaPinpoint::GetYOffset() {
  return wpi::units::millimeter_t{ReadFloat(Register::Y_POD_OFFSET)};
}

wpi::math::Pose2d GoBildaPinpoint::GetPosition() {
  return wpi::math::Pose2d{wpi::units::meter_t{GetPosX()}, wpi::units::meter_t{GetPosY()},
                     GetHeading()};
}
