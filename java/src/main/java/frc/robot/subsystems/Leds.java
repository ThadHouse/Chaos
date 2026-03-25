package frc.robot.subsystems;

import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;

public class Leds {

    private final AddressableLED m_led = new AddressableLED(3);

    private final AddressableLEDBuffer m_allRed = new AddressableLEDBuffer(30);
    private final AddressableLEDBuffer m_allGreen = new AddressableLEDBuffer(30);
    private final AddressableLEDBuffer m_allBlue = new AddressableLEDBuffer(30);

    public Leds() {
        m_led.setStart(0);
        m_led.setLength(30);
        m_led.setColorOrder(AddressableLED.ColorOrder.kRGB);

        for (int i = 0; i < m_allRed.getLength(); i++) {
            m_allRed.setRGB(i, 100, 0, 0);
            m_allGreen.setRGB(i, 0, 100, 0);
            m_allBlue.setRGB(i, 0, 0, 100);
        }
    }

    public void setAllRed() {
        m_led.setData(m_allRed);
    }

    public void setAllGreen() {
        m_led.setData(m_allGreen);
    }

    public void setAllBlue() {
        m_led.setData(m_allBlue);
    }
}
