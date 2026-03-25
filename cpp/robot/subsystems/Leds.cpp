// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Leds.h"

using namespace frc::robot::subsystems;

Leds::Leds() {
  m_led.SetStart(0);
  m_led.SetLength(kLedCount);
  m_led.SetColorOrder(frc::hardware::led::AddressableLED::ColorOrder::kRGB);

  for (int i = 0; i < kLedCount; i++) {
    m_allRed.SetRGB(i, 100, 0, 0);
    m_allGreen.SetRGB(i, 0, 100, 0);
    m_allBlue.SetRGB(i, 0, 0, 100);
  }

  m_led.Start();
}

void Leds::SetAllRed() {
  m_led.SetData(m_allRed);
}

void Leds::SetAllGreen() {
  m_led.SetData(m_allGreen);
}

void Leds::SetAllBlue() {
  m_led.SetData(m_allBlue);
}
