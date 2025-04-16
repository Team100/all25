package org.team100.lib.encoder;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/**
 * Absolute rotary position sensor using duty cycle input.
 * 
 * Duty cycle input is more robust to noise than the analog inputs, but not
 * totally immune: we saw issues in 2025 where high-frequency noise in the 5v
 * supply would produce noisy sensor output, and completely confuse the FPGA
 * counter.
 */
public abstract class DutyCycleRotaryPositionSensor extends RoboRioRotaryPositionSensor {
    private static final int kFrequencyThreshold = 500;

    private final int m_channel;
    private final DigitalInput m_digitalInput;
    private final DutyCycle m_dutyCycle;
    private final DoubleSupplier m_duty;
    private final DoubleLogger m_log_duty;
    private final IntLogger m_log_frequency;
    private final BooleanLogger m_log_connected;

    // If the encoder becomes disconnected, don't break, return the most-recent
    // value.
    // TODO: remove this, it's really up to the layers above to decide what to do.
    private double m_dutyIfDisconnected;

    protected DutyCycleRotaryPositionSensor(
            LoggerFactory parent,
            int channel,
            double inputOffset,
            EncoderDrive drive,
            boolean wrapped) {
        super(parent, inputOffset, drive, wrapped);
        LoggerFactory child = parent.child(this);
        m_channel = channel;
        m_digitalInput = new DigitalInput(channel);
        m_dutyCycle = new DutyCycle(m_digitalInput);
        m_duty = Memo.ofDouble(m_dutyCycle::getOutput);
        m_log_duty = child.doubleLogger(Level.COMP, "duty cycle");
        m_log_frequency = child.intLogger(Level.TRACE, "frequency");
        m_log_connected = child.booleanLogger(Level.TRACE, "connected");
        child.intLogger(Level.COMP, "channel").log(() -> channel);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        m_dutyCycle.close();
        m_digitalInput.close();
    }

    /**
     * Cached, almost.
     * If the encoder becomes disconnected, this returns the most-recent value until
     * the encoder becomes reconnected.
     */
    @Override
    protected OptionalDouble getRatio() {
        if (!isConnected()) {
            m_log_connected.log(() -> false);
            Util.warn(String.format("*** encoder %d not connected, returning previous value ***", m_channel));
            return OptionalDouble.of(m_dutyIfDisconnected);
        }
        m_log_connected.log(() -> true);
        double dutyCycle = m_duty.getAsDouble();
        m_log_duty.log(() -> dutyCycle);
        m_dutyIfDisconnected = dutyCycle;
        return OptionalDouble.of(dutyCycle);
    }

    private boolean isConnected() {
        int frequency = m_dutyCycle.getFrequency();
        m_log_frequency.log(() -> frequency);
        return frequency > kFrequencyThreshold;
    }
}
