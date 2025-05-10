package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Drives the LED strip attached to the Lynxmotion arm trainer, useful for
 * initial bring-up of the RoboRIO.
 * 
 * https://www.adafruit.com/product/3811
 * 
 */
public class DemoLED extends SubsystemBase {
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
        m_led.start();
    }

    public Command sweep() {
        return run(this::sweepStep);
    }

    public Command indicateCalibration(AxisCalibrator calibrator) {
        return run(() -> set((int) (calibrator.getPosition() * (LENGTH-1))));
    }

    private void set(int x) {
        for (int i = 0; i < LENGTH; ++i) {
            if (i == x) {
                m_buffer.setLED(i, Color.kOrangeRed);
            } else {
                m_buffer.setLED(i, Color.kBlack);
            }
        }
        m_led.setData(m_buffer);
    }

    private void sweepStep() {
        m_counter++;
        if (m_counter < STEPS) {
            return;
        }
        m_counter = 0;

        if (m_position == LENGTH - 1) {
            // we're at the end, so go back
            m_inc = -1;
        } else if (m_position == 0) {
            // we're at the start, so go forward
            m_inc = 1;
        }
        m_position += m_inc;
        set(m_position);
    }

}
