package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;

/**
 * Passthrough to outboard closed-loop angular control, using a profile with
 * velocity feedforward, also extra torque (e.g. for gravity). There's no
 * feedback at this level, and no feedforward calculation either, that's
 * delegated to the mechanism.
 * 
 * Must be used with a combined encoder, to "zero" the motor encoder.
 */
public class OutboardAngularPositionServo implements AngularPositionServo {
    private static final double kPositionTolerance = 0.05;
    private static final double kVelocityTolerance = 0.05;

    private final RotaryMechanism m_mechanism;
    private final ProfileReference1d m_ref;

    private final DoubleLogger m_log_goal;
    private final DoubleLogger m_log_ff_torque;
    private final DoubleLogger m_log_measurement;
    private final Control100Logger m_log_setpoint;
    private final OptionalDoubleLogger m_log_position;

    /**
     * Goal "winds up" i.e. it's it's [-inf, inf], not [-pi,pi]
     */
    private Model100 m_goal = new Model100(0, 0);
    /**
     * Setpoint "winds up" i.e. it's [-inf, inf], not [-pi,pi]
     */
    private Control100 m_setpoint = new Control100(0, 0);

    public OutboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            ProfileReference1d ref) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mech;
        m_ref = ref;
        m_log_goal = child.doubleLogger(Level.TRACE, "goal (rad)");
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.doubleLogger(Level.TRACE, "measurement (rad)");
        m_log_setpoint = child.control100Logger(Level.TRACE, "setpoint (rad)");
        m_log_position = child.optionalDoubleLogger(Level.TRACE, "Position");
    }

    @Override
    public void reset() {
        OptionalDouble position = getPosition();
        if (position.isEmpty())
            return;
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        // OptionalDouble velocity = getVelocity();
        // if (velocity.isEmpty())
        // return;
        Control100 measurement = new Control100(position.getAsDouble(), 0);
        m_setpoint = measurement;
        // TODO: do i need this somewhere else?
        // m_controller.init(new Model100(position.getAsDouble(), 0));
        // measurement is used for initialization only here.
        m_ref.setGoal(measurement.model());
        m_ref.init(measurement.model());
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_goal = null;
        m_setpoint = null;
        m_mechanism.setDutyCycle(dutyCycle);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mechanism.setTorqueLimit(torqueNm);
    }

    @Override
    public void setPositionProfiled(double goalRad, double feedForwardTorqueNm) {
        m_log_goal.log(() -> goalRad);

        Model100 goal = new Model100(mod(goalRad), 0);

        if (!goal.near(m_goal, kPositionTolerance, kVelocityTolerance)) {
            m_goal = goal;
            m_ref.setGoal(goal);
            // make sure the setpoint is near the measurement
            if (m_setpoint == null) {
                // erased by dutycycle control
                OptionalDouble posOpt = m_mechanism.getPositionRad();
                if (posOpt.isEmpty())
                    return;
                double measurement = posOpt.getAsDouble();
                m_setpoint = new Control100(measurement, 0);
            } else {
                m_setpoint = new Control100(mod(m_setpoint.x()), m_setpoint.v());
            }
            // initialize with the setpoint, not the measurement, to avoid noise.
            m_ref.init(m_setpoint.model());
        }

        actuate(m_ref.get(), feedForwardTorqueNm);
    }

    /**
     * set mechanism position passthrough, adjusting the setpoint to be close to the
     * measurement.
     * invalidates the current profile.
     * for outboard control we only use the "next" setpoint.
     */
    @Override
    public void setPositionDirect(Setpoints1d setpoint, double feedForwardTorqueNm) {
        m_goal = null;
        actuate(setpoint, feedForwardTorqueNm);
    }

    /**
     * Pass the setpoint directly to the mechanism's position controller.
     * For outboard control we only use the "next" setpoint.
     */
    private void actuate(Setpoints1d setpoint, double feedForwardTorqueNm) {
        m_setpoint = setpoint.next();
        m_mechanism.setPosition(mod(m_setpoint.x()), m_setpoint.v(), m_setpoint.a(), feedForwardTorqueNm);
        m_log_setpoint.log(() -> m_setpoint);
        m_log_ff_torque.log(() -> feedForwardTorqueNm);
    }

    /** Return an angle near the measurement */
    private double mod(double x) {
        OptionalDouble posOpt = m_mechanism.getPositionRad();
        if (posOpt.isEmpty())
            return x;
        double measurement = posOpt.getAsDouble();
        m_log_measurement.log(() -> measurement);
        return MathUtil.angleModulus(x - measurement) + measurement;
    }

    /**
     * @return the absolute 1:1 position of the mechanism in [-pi, pi]
     */
    @Override
    public OptionalDouble getPosition() {
        return m_mechanism.getPositionRad();
    }

    /**
     * Compares robotPeriodic-updated measurements to the setpoint,
     * so you need to know when the setpoint was updated: is it for the
     * current Takt time, or the next step?
     */
    @Override
    public boolean atSetpoint() {
        OptionalDouble positionRad = m_mechanism.getPositionRad();
        if (positionRad.isEmpty())
            return false;
        OptionalDouble velocityRad_S = m_mechanism.getVelocityRad_S();
        if (velocityRad_S.isEmpty())
            return false;
        double positionError = MathUtil.angleModulus(m_setpoint.x() - positionRad.getAsDouble());
        double velocityError = m_setpoint.v() - velocityRad_S.getAsDouble();
        return Math.abs(positionError) < kPositionTolerance
                && Math.abs(velocityError) < kVelocityTolerance;
    }

    @Override
    public boolean profileDone() {
        if (m_goal == null) {
            // if there's no profile, it's always done.
            return true;
        }
        return m_ref.profileDone();
    }


    /**
     * Note this is affected by the setpoint update.
     * 
     * It really makes the most sense to call this *before* updating the setpoint,
     * because the measurement comes from the recent-past Takt and the updated
     * setpoint will be aiming at the next one.
     */
    @Override
    public boolean atGoal() {
        return atSetpoint() && profileDone();
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        // m_sensor.periodic();
    }
}
