package org.team100.lib.indicator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * An LED strip used as a signal light.
 * 
 * Uses the AddressableLED feature of the RoboRIO.
 * 
 * In the past, we've used these strips: https://www.amazon.com/dp/B01CNL6LLA
 * 
 * But in 2025, we blew up several devices by, I think, putting 12 V into the
 * PWM bus of the RoboRIO, which these 12 V strips kind of invite you to do.
 * 
 * So instead, use a 5 V equivalent: https://www.amazon.com/dp/B01CDTEJBG
 * 
 * The only disadvantage of the 5 V strips is that they're not as good for long
 * runs (i.e. several meters) with many lights. For small strips, the 5v ones
 * are fine.
 * 
 * This works in simulation using the simulated camera sightings, i.e. it flips
 * from green to red as you drive around.
 */
public class LEDIndicator {
    private static final int LENGTH = 40;
    private final AddressableLED m_led;
    // buffer flipping is a little quicker than setting the pixels one at a time
    private final AddressableLEDBuffer m_greenBuffer;
    private final AddressableLEDBuffer m_redBuffer;
    private final Supplier<Double> m_timeSinceLastPose;

    public LEDIndicator(int port, Supplier<Double> timeSinceLastPose) {
        m_led = new AddressableLED(port);
        m_led.setLength(LENGTH);
        m_greenBuffer = fill(Color.kGreen);
        m_redBuffer = fill(Color.kRed);
        m_led.setData(m_redBuffer);
        m_led.start();
        m_timeSinceLastPose = timeSinceLastPose;
    }

    private static AddressableLEDBuffer fill(Color color) {
        AddressableLEDBuffer buf = new AddressableLEDBuffer(LENGTH);
        for (int i = 0; i < 40; ++i) {
            buf.setLED(i, color);
        }
        return buf;
    }

    /**
     * Periodic does all the real work in this class.
     */
    public void periodic() {
        if (m_timeSinceLastPose.get() < 1) {
            m_led.setData(m_greenBuffer);
        } else {
            m_led.setData(m_redBuffer);
        }
    }

    public void close() {
        m_led.close();
    }
}
