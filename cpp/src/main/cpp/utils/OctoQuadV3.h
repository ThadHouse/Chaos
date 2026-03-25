// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>

#include <wpi/hardware/bus/I2C.hpp>

namespace wpi {
namespace utils {

/**
 * Driver for the OctoQuad V3 multi-encoder I2C interface.
 *
 * Communicates over I2C (address 0x30) to read 8 absolute encoder positions
 * and velocities.
 */
class OctoQuadV3 {
 public:
  /** Bulk encoder data: 8 positions (counts) and 8 velocities (counts/s). */
  struct EncoderData {
    int32_t positions[8] = {};
    int16_t velocities[8] = {};
  };

  explicit OctoQuadV3(wpi::I2C::Port port);

  /**
   * Reads all encoder positions and velocities from the device.
   *
   * @param toFill EncoderData struct to populate.
   * @return True on success, false on I2C error.
   */
  bool ReadAllDataWithoutLocalizer(EncoderData& toFill);

  /**
   * Configures a single encoder channel direction.
   *
   * @param channel  Channel index (0–7).
   * @param reversed True to reverse the channel direction.
   * @return True on success, false on I2C error.
   */
  bool SetDirection(int channel, bool reversed);

  /**
   * Resets all encoder positions to zero.
   *
   * @return True on success, false on I2C error.
   */
  bool ResetAllPositions();

  /** @return Device chip ID (should be 0x51). */
  uint8_t GetChipId();

 private:
  wpi::I2C m_i2c;
  int m_encoderDirections = 0;

  /**
   * Reads bytes from a register.
   *
   * @param reg   Register address.
   * @param count Number of bytes to read.
   * @param buf   Output buffer (must be at least count bytes).
   * @return True on success, false on error.
   */
  bool ReadRegister(int reg, int count, uint8_t* buf);

  /**
   * Writes bytes to a register.
   *
   * @param reg  Register address.
   * @param data Data bytes to write.
   * @param len  Number of data bytes.
   * @return True on success, false on error.
   */
  bool WriteRegister(int reg, const uint8_t* data, int len);
};

}  // namespace utils
}  // namespace wpi
