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
 * Control position using the WPI PID controller class, wrapped in our
 * control interface.
 * 
 * This also logs the error that we have habitually logged elsewhere, because I
 * don't want the idea of "error" to be in the interface.
 */
public class PIDControllerPosWPI implements Controller100, Glassy {
    private final PIDController m_controller;
    private final DoubleLogger m_log_error;

    public PIDControllerPosWPI(
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

    /**
     * Observe position error, produce desired position and implied velo and accel.
     */
    @Override
    public Control100 calculate(Model100 measurement, Model100 setpoint) {
        double targetPos = m_controller.calculate(measurement.x(), setpoint.x());
        double targetVelo = (targetPos - measurement.x()) / TimedRobot100.LOOP_PERIOD_S;
        double targetAccel = (targetVelo - measurement.v()) / TimedRobot100.LOOP_PERIOD_S;
        m_log_error.log(m_controller::getError);
        return new Control100(targetPos, targetVelo, targetAccel);
    }

}
