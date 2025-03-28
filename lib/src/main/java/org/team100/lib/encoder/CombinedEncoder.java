package org.team100.lib.encoder;

import java.util.OptionalDouble;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.util.Memo;

import edu.wpi.first.math.MathUtil;

/**
 * Proxies an absolute sensor and an incremental sensor.
 * 
 * Synchronizes the incremental one to the absolute one, using a separate thread
 * with a delay of a few seconds.
 * 
 * Why delay? Because the dutycycle encoder seems to return slightly-wrong
 * values initially.
 * 
 * The use case is absolute + incremental encoders, in order to do outboard
 * closed-loop position control with only outboard incremental encoders --
 * RoboRIO-attached absolute encoders are the primary, and the incremental
 * encoders are the secondary. Note in this case the "primary" absolute
 * measurement is [-pi,pi] but the "secondary" measurement winds up to
 * [-inf,inf].
 * 
 * The secondary is a RotaryMechanism, instead of an encoder, because we want
 * the *gear reduction* to be applied to the underlying encoder.
 */
public class CombinedEncoder implements RotaryPositionSensor {
    private final RotaryPositionSensor m_absolute;
    private final RotaryMechanism m_incremental;
    // LOGGERS
    private final OptionalDoubleLogger m_log_absolute;
    private final OptionalDoubleLogger m_log_incremental;
    private final DoubleLogger m_log_incremental_wrapped;
    private final OptionalDoubleLogger m_log_combined;
    // for synchronization one-shot delayed task
    private final ScheduledExecutorService m_synchronizer;
    // private final boolean m_wrapped;
    private boolean m_synchronized;

    /**
     * "Zeros" the incremental sensor.
     * 
     * @param absolute    absolute sensor wired to the RoboRIO
     * @param incremental incremental sensor that needs to be "zeroed"
     */
    public CombinedEncoder(
            LoggerFactory parent,
            RotaryPositionSensor absolute,
            RotaryMechanism incremental) {
        // boolean wrapped) {
        LoggerFactory child = parent.child(this);
        m_absolute = absolute;
        m_incremental = incremental;
        m_log_absolute = child.optionalDoubleLogger(Level.DEBUG, "absolute (rad))");
        m_log_incremental = child.optionalDoubleLogger(Level.TRACE, "incremental (rad)");
        m_log_incremental_wrapped = child.doubleLogger(Level.TRACE, "incremental wrapped (rad)");
        m_log_combined = child.optionalDoubleLogger(Level.DEBUG, "combined (rad)");
        // m_wrapped = wrapped;
        // the duty cycle encoder seems to produce slightly-wrong values immediately
        // upon startup, so wait a bit before doing the synchronization
        m_synchronized = false;
        m_synchronizer = Executors.newSingleThreadScheduledExecutor();
        m_synchronizer.schedule(this::sync, 3, TimeUnit.SECONDS);
    }

    /**
     * Sync the absolute and incremental encoders.
     * 
     * Setting the encoder position is very slow, so just do it once.
     */
    void sync() {
        Memo.resetAll();
        // Assume the mechanism is stationary at startup, average a few measurements to
        // remove a little bit of noise.
        double sin = 0;
        double cos = 0;
        // 3/28/25: this used to do the mean of the measurement, which is prone to
        // errors near zero
        // double posN = 0;
        final int N = 10;
        for (int i = 0; i < N; ++i) {
            double pos = m_absolute.getPositionRad().getAsDouble();
            cos += Math.cos(pos);
            sin += Math.sin(pos);
            // posN += pos;
        }
        sin /= N;
        cos /= N;
        // posN /= N;

        double absolutePosition = Math.atan2(sin, cos);

        // if (m_wrapped) {
        // absolutePosition = posN;
        // }

        m_incremental.setEncoderPosition(absolutePosition);
        m_synchronized = true;
    }

    public boolean isSynchronized() {
        return m_synchronized;
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * The secondary (incremental motor-integrated) measurement.
     */
    @Override
    public OptionalDouble getPositionRad() {
        return m_incremental.getPositionRad();
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * The secondary (incremental motor-integrated) measurement
     */
    @Override
    public OptionalDouble getRateRad_S() {
        return m_incremental.getVelocityRad_S();
    }

    @Override
    public void close() {
        m_absolute.close();
        m_incremental.close();
    }

    /**
     * Error = Actual (i.e. absolute encoder) - Measured (i.e. motor).
     * 
     * This is used for lash correction in outboard position control.
     * 
     * Say the lash is such that the true measurement is zero, but the motor thinks
     * the measurement is 1. Then to get the motor to maintain zero, you give it a
     * positional command of 1. The error calculated here would be -1, so you have
     * to subtract the error to get the motor target.
     * 
     * (Measured = Actual - Error)
     */
    public double getError() {
        // don't attempt to compute error prior to synchronization.
        if (!m_synchronized)
            return 0;
        // The sensor reads the actual position of the mechanism.
        OptionalDouble sensorPosition = getAbsolutePositionRad();
        // The motor reads the position on the other side of the gear lash.
        OptionalDouble motorPosition = getPositionRad();
        if (sensorPosition.isEmpty() || motorPosition.isEmpty()) {
            // This is wrong but safe.
            return 0;
        }
        return sensorPosition.getAsDouble() - motorPosition.getAsDouble();
    }

    public void periodic() {
        m_log_absolute.log(m_absolute::getPositionRad);
        m_log_incremental.log(m_incremental::getPositionRad);
        m_log_incremental_wrapped.log(() -> MathUtil.angleModulus(m_incremental.getPositionRad().getAsDouble()));
        m_log_combined.log(this::getPositionRad);
    }

    /**
     * The position as read by the absolute sensor. This is useful for debugging and
     * for the lash observer. Don't use this unless you know what you're doing.
     */
    OptionalDouble getAbsolutePositionRad() {
        return m_absolute.getPositionRad();
    }
}