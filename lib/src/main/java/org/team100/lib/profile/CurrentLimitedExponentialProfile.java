package org.team100.lib.profile;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

/**
 * Uses a trapezoid profile for low (current-limited) speed.
 * Uses an exponential profile for high (back-EMF-limited) speed.
 * 
 * The trapezoid profile maximum acceleration relates to the current-limited
 * torque.
 * 
 * The exponential profile maximum acceleration relates to the *unlimited* stall
 * torque.
 */
public class CurrentLimitedExponentialProfile implements Profile100 {
    private static final boolean DEBUG = false;
    private final double m_maxVel;
    /** Unlimited stall torque */
    private final double m_stallAccel;
    /** Current-limited torque */
    private final double m_limitedAccel;
    private final TrapezoidProfileWPI m_trapezoid;
    private final ExponentialProfileWPI m_exponential;
    /** Speed where the torque curves cross */
    private final double m_limit;

    public CurrentLimitedExponentialProfile(
            double maxVel,
            double limitedAccel,
            double stallAccel) {
        m_maxVel = maxVel;
        m_stallAccel = stallAccel;
        m_limitedAccel = limitedAccel;
        m_trapezoid = new TrapezoidProfileWPI(maxVel, limitedAccel);
        m_exponential = new ExponentialProfileWPI(maxVel, stallAccel);
        m_limit = (1 - m_limitedAccel/m_stallAccel) * maxVel;
    }

    boolean isAccel(Model100 initial, Control100 setpoint) {
        double initialV = initial.v();
        double setpointV = setpoint.v();
        boolean isAccel = Math.abs(setpointV) > Math.abs(initialV);
        if (DEBUG)
            Util.printf("initial %5.2f setpoint %5.2f isAccel %b\n", initialV, setpointV, isAccel);
        return isAccel;
    }

    @Override
    public Control100 calculate(double dt, Model100 initial, Model100 goal) {
        Control100 trapezoid = m_trapezoid.calculate(dt, initial, goal);
        Control100 exponential = m_exponential.calculate(dt, initial, goal);
        if (!isAccel(initial, exponential)) {
            // exponential decel is more accurate ("plugging" torque is higher than stall)
            return exponential;
        }
        if (initial.v() < m_limit) {
            // Low speed is current limited.
            return trapezoid;
        }
        // high speed is back-EMF limited.
        return exponential;
    }

    @Override
    public ResultWithETA calculateWithETA(double dt, Model100 initial, Model100 goal) {
        ResultWithETA trapezoid = m_trapezoid.calculateWithETA(dt, initial, goal);
        ResultWithETA exponential = m_exponential.calculateWithETA(dt, initial, goal);
        if (!isAccel(initial, exponential.state())) {
            // exponential decel is more accurate ("plugging" torque is higher than stall)
            return exponential;
        }

        if (initial.v() < m_limit) {
            // Low speed is current limited.
            return trapezoid;
        }
        // high speed is back-EMF limited.
        return exponential;
    }

    @Override
    public Profile100 scale(double s) {
        return new CurrentLimitedExponentialProfile(
                m_maxVel, s * m_limitedAccel, s * m_stallAccel);
    }

    @Override
    public double getMaxVelocity() {
        return m_maxVel;
    }

}
