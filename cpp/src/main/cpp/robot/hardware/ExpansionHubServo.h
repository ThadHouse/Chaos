// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>

namespace frc {
namespace robot {
namespace hardware {

/**
 * Controls a servo connected to an ExpansionHub over NetworkTables.
 *
 * Mirrors the Java org.wpilib.hardware.expansionhub.ExpansionHubServo class,
 * communicating with the SystemCore system server via the same NT topics:
 *   /rhsp/{usbId}/servo{channel}/...
 *
 * Operates in continuous-rotation mode by default (Set() accepts -1 to 1).
 */
class ExpansionHubServo {
 public:
  /**
   * Constructs a servo controller for the given hub and channel.
   *
   * @param usbId   USB port ID of the hub (0-based).
   * @param channel Servo channel on the hub (0-based).
   */
  ExpansionHubServo(int usbId, int channel);

  ExpansionHubServo(const ExpansionHubServo&) = delete;
  ExpansionHubServo& operator=(const ExpansionHubServo&) = delete;

  /**
   * Sets the servo output.
   *
   * In continuous-rotation mode (default), value is in [-1, 1] where the
   * sign indicates direction and magnitude indicates speed.
   *
   * @param value Output value.
   */
  void Set(double value);

  /** Enables or disables the servo output. */
  void SetEnabled(bool enabled);

 private:
  static constexpr int kDefaultMinPwm = 600;
  static constexpr int kDefaultMaxPwm = 2400;
  static constexpr int kDefaultFramePeriodUs = 20000;

  nt::IntegerPublisher m_pulseWidthPublisher;
  nt::IntegerPublisher m_framePeriodPublisher;
  nt::BooleanPublisher m_enabledPublisher;
};

}  // namespace hardware
}  // namespace robot
}  // namespace frc
