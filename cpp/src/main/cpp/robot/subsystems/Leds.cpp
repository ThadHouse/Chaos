// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Leds.h"

#include <span>

using namespace wpi::robot::subsystems;

Leds::Leds()
    : m_allRed(kLedCount),
      m_allGreen(kLedCount),
      m_allBlue(kLedCount) {
  m_led.SetStart(0);
  m_led.SetLength(kLedCount);
  m_led.SetColorOrder(wpi::AddressableLED::ColorOrder::kRGB);

  for (int i = 0; i < kLedCount; i++) {
    m_allRed[i].SetRGB(100, 0, 0);
    m_allGreen[i].SetRGB(0, 100, 0);
    m_allBlue[i].SetRGB(0, 0, 100);
  }
}

void Leds::SetAllRed() {
  m_led.SetData(std::span<const wpi::AddressableLED::LEDData>{m_allRed});
}

void Leds::SetAllGreen() {
  m_led.SetData(std::span<const wpi::AddressableLED::LEDData>{m_allGreen});
}

void Leds::SetAllBlue() {
  m_led.SetData(std::span<const wpi::AddressableLED::LEDData>{m_allBlue});
}
