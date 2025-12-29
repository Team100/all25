package org.team100.frc2025.indicator;

import org.team100.frc2025.Climber.ClimberIntake;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.NudgingVisionUpdater;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
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
    private static final double BLINK_INTERVAL_S = 0.05;

    private final AddressableLED m_led;
    // buffer flipping is a little quicker than setting the pixels one at a time
    private final AddressableLEDBuffer m_greenBuffer;
    private final AddressableLEDBuffer m_redBuffer;
    private final AddressableLEDBuffer m_tealBuffer;
    private final AddressableLEDBuffer m_orangeBuffer;
    private final AddressableLEDBuffer m_blackBuffer;
    private final AddressableLEDBuffer m_whiteBuffer;

    private final NudgingVisionUpdater m_updater;
    private final Manipulator m_manipulator;
    private final ClimberIntake m_climberIntake;

    private double m_lastBlinkTime = 0;
    private boolean m_blinkState = false;

    /**
     * The constructor should take *subsystems* not *buttons.*
     * 
     * Since this has logic about the 2025 game it should be in the "comp" project.
     */
    public LEDIndicator(
            NudgingVisionUpdater updater,
            Manipulator manipulator,
            ClimberIntake climberIntake) {
        m_led = new AddressableLED(0);
        m_led.setLength(LENGTH);
        m_greenBuffer = fill(Color.kGreen);
        m_tealBuffer = fill(Color.kTeal);
        m_whiteBuffer = fill(Color.kWhiteSmoke);
        m_orangeBuffer = fill(Color.kOrangeRed);
        m_redBuffer = fill(Color.kRed);
        m_blackBuffer = fill(Color.kBlack);
        m_led.setData(m_redBuffer);
        m_led.start();
        m_updater = updater;
        m_manipulator = manipulator;
        m_climberIntake = climberIntake;
    }

    /**
     * Periodic does all the real work in this class.
     */
    public void periodic() {

        if (RobotState.isDisabled()) {
            if (m_updater.getPoseAgeSec() < 1) {
                m_led.setData(m_greenBuffer);
            } else {
                m_led.setData(m_redBuffer);
            }
        } else if (m_climberIntake.isIn()) {
            if (shouldBlink()) {
                m_led.setData(m_blinkState ? m_greenBuffer : m_blackBuffer);
            }
        } else if (m_manipulator.hasCoral() || m_manipulator.hasCoralSideways()) {
            if (m_manipulator.hasAlgae()) {
                if (shouldBlink()) {
                    m_led.setData(m_blinkState ? m_tealBuffer : m_whiteBuffer);
                }
            } else {
                m_led.setData(m_whiteBuffer);
            }
        } else if (m_manipulator.hasAlgae()) {
            m_led.setData(m_tealBuffer);
        } else if (m_manipulator.intakingAlgae()) {
            if (shouldBlink()) {
                m_led.setData(m_blinkState ? m_tealBuffer : m_orangeBuffer);
            }
        } else if (m_manipulator.intakingCoral()) {
            if (shouldBlink()) {
                m_led.setData(m_blinkState ? m_whiteBuffer : m_orangeBuffer);
            }
        } else {
            m_led.setData(m_orangeBuffer);
        }
    }

    public void close() {
        m_led.close();
    }

    ///////////////////////////////////////////////////////////

    /**
     * Handles blink timing and returns true if blink state changed
     */
    private boolean shouldBlink() {
        double currentTime = Takt.actual();
        if (currentTime - m_lastBlinkTime > BLINK_INTERVAL_S) {
            m_blinkState = !m_blinkState;
            m_lastBlinkTime = currentTime;
            return true;
        }
        return false;
    }

    private static AddressableLEDBuffer fill(Color color) {
        AddressableLEDBuffer buf = new AddressableLEDBuffer(LENGTH);
        for (int i = 0; i < LENGTH; ++i) {
            buf.setLED(i, color);
        }
        return buf;
    }
}