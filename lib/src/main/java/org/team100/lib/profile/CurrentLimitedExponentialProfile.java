package org.team100.lib.profile;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Uses a trapezoid profile for low (current-limited) speed.
 * Uses an exponential profile for high (back-EMF-limited) speed.
 */
public class CurrentLimitedExponentialProfile implements Profile100 {

    private final double m_maxVel;
    private final double m_maxAccel;
    private final TrapezoidProfileWPI m_trapezoid;
    private final ExponentialProfileWPI m_exponential;
    private final double m_limit;

    public CurrentLimitedExponentialProfile(double maxVel, double maxAccel, double limit) {
        m_maxVel = maxVel;
        m_maxAccel = maxAccel;
        m_trapezoid = new TrapezoidProfileWPI(maxVel, maxAccel);
        m_exponential = new ExponentialProfileWPI(maxVel, maxAccel);
        m_limit = limit;
    }

    @Override
    public Control100 calculate(double dt, Model100 initial, Model100 goal) {
        if (initial.v() > m_limit)
            return m_exponential.calculate(dt, initial, goal);
        return m_trapezoid.calculate(dt, initial, goal);
    }

    @Override
    public ResultWithETA calculateWithETA(double dt, Model100 initial, Model100 goal) {
        if (initial.v() > m_limit)
            return m_exponential.calculateWithETA(dt, initial, goal);
        return m_trapezoid.calculateWithETA(dt, initial, goal);
    }

    @Override
    public Profile100 scale(double s) {
        return new CurrentLimitedExponentialProfile(m_maxVel, s * m_maxAccel, m_limit);
    }

    @Override
    public double getMaxVelocity() {
        return m_maxVel;
    }

}
