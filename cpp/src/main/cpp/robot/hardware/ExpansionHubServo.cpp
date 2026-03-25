// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ExpansionHubServo.h"

#include <algorithm>
#include <string>

#include <ntcore_cpp.h>

using namespace frc::robot::hardware;

static nt::PubSubOptions MakeOptions() {
  nt::PubSubOptions opts;
  opts.sendAll = true;
  opts.keepDuplicates = true;
  opts.periodic = 0.005;
  return opts;
}

ExpansionHubServo::ExpansionHubServo(int usbId, int channel) {
  auto& inst = nt::NetworkTableInstance::GetDefault();
  auto opts = MakeOptions();
  std::string prefix = "/rhsp/" + std::to_string(usbId) + "/servo" +
                       std::to_string(channel);

  m_pulseWidthPublisher =
      inst.GetIntegerTopic(prefix + "/pulseWidth").Publish(opts);
  m_framePeriodPublisher =
      inst.GetIntegerTopic(prefix + "/framePeriod").Publish(opts);
  m_enabledPublisher = inst.GetBooleanTopic(prefix + "/enabled").Publish(opts);

  m_pulseWidthPublisher.Set(1500);
  m_framePeriodPublisher.Set(kDefaultFramePeriodUs);
}

void ExpansionHubServo::Set(double value) {
  // Continuous-rotation mode: map [-1, 1] to [0, 1] then to PWM range
  value = std::clamp(value, -1.0, 1.0);
  value = (value + 1.0) / 2.0;
  int rawValue =
      static_cast<int>(value * (kDefaultMaxPwm - kDefaultMinPwm)) +
      kDefaultMinPwm;
  SetEnabled(true);
  m_pulseWidthPublisher.Set(rawValue);
}

void ExpansionHubServo::SetEnabled(bool enabled) {
  m_enabledPublisher.Set(enabled);
}
