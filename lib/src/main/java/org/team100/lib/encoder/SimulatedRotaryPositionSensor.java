package org.team100.lib.encoder;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.util.Takt;

import edu.wpi.first.math.MathUtil;

/**
 * Integrates mechanism velocity to find position. If you use this in tests,
 * you'll have to control the clock somehow, e.g. by using {@link Timeless}.
 */
public class SimulatedRotaryPositionSensor implements RotaryPositionSensor {
    private final RotaryMechanism m_mechanism;
    private final DoubleSupplier m_lash;
    // LOGGERS
    private final DoubleLogger m_log_position;
    private final OptionalDoubleLogger m_log_rate;

    private double m_positionRad = 0;
    // to calculate the position with trapezoid integral
    private double m_previousVelocity = 0;
    private double m_timeS = Takt.get();

    public SimulatedRotaryPositionSensor(
            LoggerFactory parent,
            RotaryMechanism mechanism,
            DoubleSupplier lash) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mechanism;
        m_lash = lash;
        m_log_position = child.doubleLogger(Level.TRACE, "position");
        m_log_rate = child.optionalDoubleLogger(Level.TRACE, "rate");
    }

    /**
     * Integrates the mechanism velocity between the previous call and the current
     * instant.
     */
    @Override
    public OptionalDouble getPositionRad() {
        double nowS = Takt.get();
        double dtS = nowS - m_timeS;
        // this is the velocity at the current instant.
        // motor velocity is rad/s
        OptionalDouble velocityRad_S = m_mechanism.getVelocityRad_S();
        if (velocityRad_S.isEmpty())
            return OptionalDouble.empty();

        // use the previous velocity to calculate the trapezoidal integral
        m_positionRad += 0.5 * (velocityRad_S.getAsDouble() + m_previousVelocity) * dtS;
        m_previousVelocity = velocityRad_S.getAsDouble();

        m_positionRad = MathUtil.angleModulus(m_positionRad);
        m_timeS = nowS;
        m_log_position.log(() -> m_positionRad);
        return OptionalDouble.of(m_positionRad + m_lash.getAsDouble());
    }

    @Override
    public OptionalDouble getRateRad_S() {
        // motor velocity is rad/s
        OptionalDouble m_rate = m_mechanism.getVelocityRad_S();
        if (m_rate.isEmpty())
            return OptionalDouble.empty();
        m_log_rate.log(() -> m_rate);
        return m_rate;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        //
    }

}
