// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Leds.h"

#include <span>

using namespace frc::robot::subsystems;

Leds::Leds()
    : m_allRed(kLedCount),
      m_allGreen(kLedCount),
      m_allBlue(kLedCount) {
  m_led.SetLength(kLedCount);

  for (int i = 0; i < kLedCount; i++) {
    m_allRed[i].SetRGB(100, 0, 0);
    m_allGreen[i].SetRGB(0, 100, 0);
    m_allBlue[i].SetRGB(0, 0, 100);
  }

  m_led.Start();
}

void Leds::SetAllRed() {
  m_led.SetData(std::span<const frc::AddressableLED::LEDData>{m_allRed});
}

void Leds::SetAllGreen() {
  m_led.SetData(std::span<const frc::AddressableLED::LEDData>{m_allGreen});
}

void Leds::SetAllBlue() {
  m_led.SetData(std::span<const frc::AddressableLED::LEDData>{m_allBlue});
}
