// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <units/current.h>

namespace frc {
namespace robot {
namespace hardware {

/**
 * Controls a motor connected to an ExpansionHub over NetworkTables.
 *
 * Mirrors the Java org.wpilib.hardware.expansionhub.ExpansionHubMotor class,
 * communicating with the SystemCore system server via the same NT topics:
 *   /rhsp/{usbId}/motor{channel}/...
 */
class ExpansionHubMotor {
 public:
  /**
   * Constructs a motor controller for the given hub and channel.
   *
   * @param usbId   USB port ID of the hub (0-based).
   * @param channel Motor channel on the hub (0-based).
   */
  ExpansionHubMotor(int usbId, int channel);

  ExpansionHubMotor(const ExpansionHubMotor&) = delete;
  ExpansionHubMotor& operator=(const ExpansionHubMotor&) = delete;

  /** Sets the motor to run at the given velocity setpoint (in distance units/s). */
  void SetVelocitySetpoint(double setpoint);

  /** Sets the motor enabled state. */
  void SetEnabled(bool enabled);

  /** Reverses the motor and encoder direction. */
  void SetReversed(bool reversed);

  /** Resets the encoder position to zero. */
  void ResetEncoder();

  /** Sets the distance traveled per encoder count. */
  void SetDistancePerCount(double perCount);

  /** @return Encoder position in distance units. */
  double GetEncoderPosition() const;

  /** @return Encoder velocity in distance units per second. */
  double GetEncoderVelocity() const;

  /** @return Motor current draw in amps. */
  units::ampere_t GetCurrent() const;

  /** @return True if the hub is connected. */
  bool IsHubConnected() const;

  /**
   * Velocity PID constants helper class.
   *
   * Mirrors org.wpilib.hardware.expansionhub.ExpansionHubPidConstants.
   */
  class VelocityPidConstants {
   public:
    VelocityPidConstants(int usbId, int channel);
    void SetPID(double p, double i, double d);
    void SetFF(double s, double v, double a);

   private:
    nt::DoublePublisher m_pPublisher;
    nt::DoublePublisher m_iPublisher;
    nt::DoublePublisher m_dPublisher;
    nt::DoublePublisher m_sPublisher;
    nt::DoublePublisher m_vPublisher;
    nt::DoublePublisher m_aPublisher;
  };

  /** @return Mutable reference to velocity PID constants. */
  VelocityPidConstants& GetVelocityPidConstants();

 private:
  static constexpr int kVelocityMode = 3;

  nt::DoubleSubscriber m_encoderSubscriber;
  nt::DoubleSubscriber m_encoderVelocitySubscriber;
  nt::DoubleSubscriber m_currentSubscriber;

  nt::DoublePublisher m_setpointPublisher;
  nt::DoublePublisher m_distancePerCountPublisher;
  nt::BooleanPublisher m_enabledPublisher;
  nt::BooleanPublisher m_reversedPublisher;
  nt::BooleanPublisher m_resetEncoderPublisher;
  nt::IntegerPublisher m_modePublisher;

  VelocityPidConstants m_velocityPidConstants;
};

}  // namespace hardware
}  // namespace robot
}  // namespace frc
