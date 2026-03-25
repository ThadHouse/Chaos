// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <wpi/hardware/led/AddressableLED.hpp>

namespace wpi {
namespace robot {
namespace subsystems {

/**
 * Controls an addressable LED strip (PWM port 3, 30 LEDs).
 *
 * Pre-computes red, green, and blue buffers for fast color switching.
 * Used by the Shooter subsystem to give visual velocity feedback.
 */
class Leds {
 public:
  Leds();

  /** Set all LEDs to red. */
  void SetAllRed();

  /** Set all LEDs to green. */
  void SetAllGreen();

  /** Set all LEDs to blue. */
  void SetAllBlue();

 private:
  static constexpr int kLedPort = 3;
  static constexpr int kLedCount = 30;

  wpi::AddressableLED m_led{kLedPort};
  std::vector<wpi::AddressableLED::LEDData> m_allRed;
  std::vector<wpi::AddressableLED::LEDData> m_allGreen;
  std::vector<wpi::AddressableLED::LEDData> m_allBlue;
};

}  // namespace subsystems
}  // namespace robot
}  // namespace wpi
