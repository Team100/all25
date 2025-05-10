package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Drives the LED strip attached to the Lynxmotion arm trainer, useful for
 * initial bring-up of the RoboRIO.
 * 
 * https://www.adafruit.com/product/3811
 * 
 */
public class DemoLED {
    private static final int LENGTH = 30;
    private final int STEPS = 2;
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private int m_position = 0;
    private int m_inc = 1;
    private int m_counter = 0;

    public DemoLED() {
        m_led = new AddressableLED(9);
        m_buffer = new AddressableLEDBuffer(LENGTH);
        m_led.setLength(LENGTH);
        update();
        m_led.start();
    }

    public void periodic() {
        m_counter++;
        if (m_counter < STEPS) {
            return;
        }
        m_counter = 0;
        update();
    }

    private void update() {
        for (int i = 0; i < LENGTH; ++i) {
            if (i == m_position) {
                m_buffer.setLED(i, Color.kOrangeRed);
            } else {
                m_buffer.setLED(i, Color.kBlack);
            }
        }
        m_led.setData(m_buffer);

        if (m_position == LENGTH - 1) {
            // we're at the end, so go back
            m_inc = -1;
        } else if (m_position == 0) {
            // we're at the start, so go forward
            m_inc = 1;
        }
        m_position += m_inc;
    }

}
