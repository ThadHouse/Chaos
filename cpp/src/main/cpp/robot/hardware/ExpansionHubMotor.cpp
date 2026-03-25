// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ExpansionHubMotor.h"

#include <string>

#include <ntcore_cpp.h>

using namespace frc::robot::hardware;

// ---------------------------------------------------------------------------
// Shared options for all publishers/subscribers (mirrors Java PubSubOption usage)
// ---------------------------------------------------------------------------
static nt::PubSubOptions MakeOptions() {
  nt::PubSubOptions opts;
  opts.sendAll = true;
  opts.keepDuplicates = true;
  opts.periodic = 0.005;
  return opts;
}

// ---------------------------------------------------------------------------
// VelocityPidConstants
// ---------------------------------------------------------------------------
ExpansionHubMotor::VelocityPidConstants::VelocityPidConstants(int usbId,
                                                               int channel) {
  auto& inst = nt::NetworkTableInstance::GetDefault();
  auto opts = MakeOptions();
  std::string prefix = "/rhsp/" + std::to_string(usbId) + "/motor" +
                       std::to_string(channel) + "/velocityPid";
  m_pPublisher = inst.GetDoubleTopic(prefix + "/p").Publish(opts);
  m_iPublisher = inst.GetDoubleTopic(prefix + "/i").Publish(opts);
  m_dPublisher = inst.GetDoubleTopic(prefix + "/d").Publish(opts);
  m_sPublisher = inst.GetDoubleTopic(prefix + "/s").Publish(opts);
  m_vPublisher = inst.GetDoubleTopic(prefix + "/v").Publish(opts);
  m_aPublisher = inst.GetDoubleTopic(prefix + "/a").Publish(opts);
}

void ExpansionHubMotor::VelocityPidConstants::SetPID(double p, double i,
                                                      double d) {
  m_pPublisher.Set(p);
  m_iPublisher.Set(i);
  m_dPublisher.Set(d);
}

void ExpansionHubMotor::VelocityPidConstants::SetFF(double s, double v,
                                                     double a) {
  m_sPublisher.Set(s);
  m_vPublisher.Set(v);
  m_aPublisher.Set(a);
}

// ---------------------------------------------------------------------------
// ExpansionHubMotor
// ---------------------------------------------------------------------------
ExpansionHubMotor::ExpansionHubMotor(int usbId, int channel)
    : m_velocityPidConstants(usbId, channel) {
  auto& inst = nt::NetworkTableInstance::GetDefault();
  auto opts = MakeOptions();
  std::string prefix = "/rhsp/" + std::to_string(usbId) + "/motor" +
                       std::to_string(channel);

  m_encoderSubscriber =
      inst.GetDoubleTopic(prefix + "/encoder").Subscribe(0.0, opts);
  m_encoderVelocitySubscriber =
      inst.GetDoubleTopic(prefix + "/encoderVelocity").Subscribe(0.0, opts);
  m_currentSubscriber =
      inst.GetDoubleTopic(prefix + "/current").Subscribe(0.0, opts);

  m_setpointPublisher =
      inst.GetDoubleTopic(prefix + "/setpoint").Publish(opts);
  m_distancePerCountPublisher =
      inst.GetDoubleTopic(prefix + "/distancePerCount").Publish(opts);
  m_enabledPublisher =
      inst.GetBooleanTopic(prefix + "/enabled").Publish(opts);
  m_reversedPublisher =
      inst.GetBooleanTopic(prefix + "/reversed").Publish(opts);
  m_resetEncoderPublisher =
      inst.GetBooleanTopic(prefix + "/resetEncoder").Publish(opts);
  m_modePublisher = inst.GetIntegerTopic(prefix + "/mode").Publish(opts);
}

void ExpansionHubMotor::SetVelocitySetpoint(double setpoint) {
  SetEnabled(true);
  m_modePublisher.Set(kVelocityMode);
  m_setpointPublisher.Set(setpoint);
}

void ExpansionHubMotor::SetEnabled(bool enabled) {
  m_enabledPublisher.Set(enabled);
}

void ExpansionHubMotor::SetReversed(bool reversed) {
  m_reversedPublisher.Set(reversed);
}

void ExpansionHubMotor::ResetEncoder() {
  m_resetEncoderPublisher.Set(true);
}

void ExpansionHubMotor::SetDistancePerCount(double perCount) {
  m_distancePerCountPublisher.Set(perCount);
}

double ExpansionHubMotor::GetEncoderPosition() const {
  return m_encoderSubscriber.Get(0.0);
}

double ExpansionHubMotor::GetEncoderVelocity() const {
  return m_encoderVelocitySubscriber.Get(0.0);
}

units::ampere_t ExpansionHubMotor::GetCurrent() const {
  return units::ampere_t{m_currentSubscriber.Get(0.0)};
}

bool ExpansionHubMotor::IsHubConnected() const {
  // If the NT topic is being published by the system server, the hub is connected
  return m_encoderSubscriber.GetTopic().Exists();
}

ExpansionHubMotor::VelocityPidConstants&
ExpansionHubMotor::GetVelocityPidConstants() {
  return m_velocityPidConstants;
}
