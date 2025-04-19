package org.team100.lib.controller.simple;

import org.team100.lib.state.Model100;

/**
 * A very simple feedback method, proportional to position error.
 * 
 * This is really intended just for tests.
 */
public class PositionProportionalFeedback implements Feedback100 {
    private final double m_p;
    private final double m_tol;
    private boolean m_atSetpoint;

    public PositionProportionalFeedback(double p, double tol) {
        m_p = p;
        m_tol = tol;
    }

    @Override
    public double calculate(Model100 measurement, Model100 setpoint) {
        double xError = setpoint.x() - measurement.x();
        m_atSetpoint = Math.abs(xError) < m_tol;
        return m_p * xError;
    }

    @Override
    public boolean atSetpoint() {
        return m_atSetpoint;
    }

    @Override
    public void reset() {
        m_atSetpoint = false;
    }

}
