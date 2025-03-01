package org.team100.lib.encoder;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/**
 * Absolute rotary position sensor using duty cycle input.
 */
public abstract class DutyCycleRotaryPositionSensor extends RoboRioRotaryPositionSensor {
    private static final int kFrequencyThreshold = 1000;

    private final int m_channel;
    private final DigitalInput m_digitalInput;
    private final DutyCycle m_dutyCycle;
    // CACHES
    private final DoubleSupplier m_duty;
    // LOGGERS
    private final DoubleLogger m_log_duty;

    // if the encoder becomes disconnected, don't break, return the most-recent
    // value.
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
        m_log_duty = child.doubleLogger(Level.TRACE, "duty cycle");
        child.intLogger(Level.COMP, "channel").log(() -> channel);
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
            Util.warn(String.format("*** encoder %d not connected, returning previous value ***", m_channel));
            return OptionalDouble.of(m_dutyIfDisconnected);
        }
        double dutyCycle = m_duty.getAsDouble();
        m_log_duty.log(() -> dutyCycle);
        m_dutyIfDisconnected = dutyCycle;
        return OptionalDouble.of(dutyCycle);
    }

    private boolean isConnected() {
        return m_dutyCycle.getFrequency() > kFrequencyThreshold;
    }
}
