package org.team100.lib.controller.simple;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.state.Model100;

/**
 * Feedback that is always zero: this is for use with outboard servos, where
 * feedback control is implemented in the motor controller.
 */
public class ZeroFeedback implements Feedback100 {
    private final DoubleUnaryOperator m_modulus;
    private final double m_xtol;
    private final double m_vtol;
    
    private boolean m_atSetpoint = false;

    public ZeroFeedback(DoubleUnaryOperator modulus, double xtol, double vtol) {
        m_modulus = modulus;
        m_xtol = xtol;
        m_vtol = vtol;
    }

    @Override
    public double calculate(Model100 measurement, Model100 setpoint) {
        double xError = m_modulus.applyAsDouble(setpoint.x() - measurement.x());
        double xDotError = setpoint.v() - measurement.v();
        m_atSetpoint = Math.abs(xError) < m_xtol && Math.abs(xDotError) < m_vtol;
        return 0;
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
