// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OctoQuadV3.h"

#include <cstring>
#include <iostream>
#include <vector>

using namespace wpi::utils;

OctoQuadV3::OctoQuadV3(wpi::I2C::Port port)
    : m_i2c{port, 0x30} {
  uint8_t chipId = GetChipId();
  if (chipId != 0x51) {
    std::cout << "OctoQuadV3: Invalid chip ID: 0x" << std::hex
              << static_cast<int>(chipId) << std::dec << "\n";
  }
}

bool OctoQuadV3::ReadRegister(int reg, int count, uint8_t* buf) {
  if (m_i2c.Read(reg, count, buf)) {
    std::cout << "OctoQuadV3: Error reading register: " << reg << "\n";
    return false;
  }
  return true;
}

bool OctoQuadV3::WriteRegister(int reg, const uint8_t* data, int len) {
  std::vector<uint8_t> packet(len + 1);
  packet[0] = static_cast<uint8_t>(reg);
  std::memcpy(packet.data() + 1, data, len);

  if (m_i2c.WriteBulk(packet.data(), static_cast<int>(packet.size()))) {
    std::cout << "OctoQuadV3: Error writing register: " << reg << "\n";
    return false;
  }
  return true;
}

uint8_t OctoQuadV3::GetChipId() {
  uint8_t val = 0;
  if (!ReadRegister(0x00, 1, &val)) {
    return 0;
  }
  return val;
}

bool OctoQuadV3::ResetAllPositions() {
  uint8_t data[] = {21, static_cast<uint8_t>(0xFF)};
  return WriteRegister(0x04, data, 2);
}

bool OctoQuadV3::SetDirection(int channel, bool reversed) {
  if (reversed) {
    m_encoderDirections |= (1 << channel);
  } else {
    m_encoderDirections &= ~(1 << channel);
  }
  uint8_t data[] = {1, 0, static_cast<uint8_t>(m_encoderDirections)};
  return WriteRegister(0x04, data, 3);
}

bool OctoQuadV3::ReadAllDataWithoutLocalizer(EncoderData& toFill) {
  // 8 positions × 4 bytes + 8 velocities × 2 bytes = 48 bytes total
  constexpr int kNumRead = (8 * 4) + (8 * 2);
  uint8_t data[kNumRead];

  if (m_i2c.Read(0x1C, kNumRead, data)) {
    std::cout << "OctoQuadV3: Failed to read data\n";
    return false;
  }

  // Parse 8 little-endian int32 positions
  for (int i = 0; i < 8; i++) {
    std::memcpy(&toFill.positions[i], data + (i * 4), 4);
  }

  // Parse 8 little-endian int16 velocities
  for (int i = 0; i < 8; i++) {
    std::memcpy(&toFill.velocities[i], data + (8 * 4) + (i * 2), 2);
  }

  return true;
}
