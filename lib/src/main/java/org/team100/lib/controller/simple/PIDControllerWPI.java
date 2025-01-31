package org.team100.lib.controller.simple;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.controller.PIDController;

/**
 * Wrap the WPI PID controller in our control interface.
 * This also logs the error that we have habitually logged elsewhere, because I
 * don't want the idea of "error" to be in the interface.
 */
public class PIDControllerWPI implements Controller100, Glassy {
    private final PIDController m_controller;
    private final DoubleLogger m_log_error;

    public PIDControllerWPI(
            LoggerFactory parent,
            double p,
            double i,
            double d,
            boolean rotation) {
        m_controller = new PIDController(p, i, d, TimedRobot100.LOOP_PERIOD_S);
        if (rotation)
            m_controller.enableContinuousInput(-Math.PI, Math.PI);
        LoggerFactory child = parent.child(this);
        m_log_error = child.doubleLogger(Level.TRACE, "error");
    }

    @Override
    public Control100 calculate(Model100 measurement, Model100 setpoint) {
        double u = m_controller.calculate(measurement.x(), setpoint.x());
        m_log_error.log(m_controller::getError);
        // return "u" in the "a" slot.
        // TODO: not that?
        return new Control100(0, 0, u);
    }

}
