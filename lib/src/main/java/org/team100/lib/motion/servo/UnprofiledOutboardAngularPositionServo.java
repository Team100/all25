package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;

/**
 * Passthrough to outboard closed-loop angular control, without a profile, i.e.
 * it's just a position command.
 * 
 */
public class UnprofiledOutboardAngularPositionServo implements AngularPositionServo {
    private static final double kPositionTolerance = 0.05;
    private static final double kVelocityTolerance = 0.05;

    private final RotaryMechanism m_mechanism;
    private final CombinedEncoder m_encoder;

    // LOGGERS
    private final Model100Logger m_log_goal;
    private final DoubleLogger m_log_ff_torque;
    private final DoubleLogger m_log_measurement;

    /**
     * Goal "winds up" i.e. it's it's [-inf, inf], not [-pi,pi]
     */
    private Model100 m_goal = new Model100(0, 0);

    public UnprofiledOutboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            CombinedEncoder encoder) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mech;
        m_encoder = encoder;
        m_log_goal = child.model100Logger(Level.TRACE, "goal (rad)");
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.doubleLogger(Level.TRACE, "measurement (rad)");
    }

    @Override
    public void reset() {
        m_mechanism.resetEncoderPosition();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mechanism.setTorqueLimit(torqueNm);
    }

    @Override
    public void setEncoderPosition(double value) {
        m_mechanism.setEncoderPosition(value);
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        m_mechanism.setDutyCycle(dutyCycle);
    }

    /**
     * Sets the goal, updates the setpoint to the "next step" value towards it,
     * gives the setpoint to the outboard mechanism.
     * 
     * The outboard measurement does not wrap, but the goal does.
     * 
     * @param goalRad             [-pi, pi]
     * @param goalVelocityRad_S
     * @param feedForwardTorqueNm
     */
    @Override
    public void setPositionWithVelocity(double goalRad, double goalVelocityRad_S, double feedForwardTorqueNm) {
        OptionalDouble posOpt = m_encoder.getPositionRad();
        if (posOpt.isEmpty())
            return;

        // measurement is [-inf,inf]
        final double measurement = posOpt.getAsDouble();

        // choose a goal which is near the measurement
        // goal is [-inf, inf]
        m_goal = new Model100(MathUtil.angleModulus(goalRad - measurement) + measurement,
                goalVelocityRad_S);

        if (Experiments.instance.enabled(Experiment.LashCorrection)) {
            double lashError = m_encoder.getError();
            m_mechanism.setPosition(m_goal.x() - lashError, m_goal.v(), 0, feedForwardTorqueNm);
        } else {
            m_mechanism.setPosition(m_goal.x(), m_goal.v(), 0, feedForwardTorqueNm);
        }
        m_log_goal.log(() -> m_goal);
        m_log_ff_torque.log(() -> feedForwardTorqueNm);
        m_log_measurement.log(() -> measurement);
    }

    @Override
    public void setPosition(double goal, double feedForwardTorqueNm) {
        setPositionWithVelocity(goal, 0.0, feedForwardTorqueNm);
    }

    /** Value is updated in Robot.robotPeriodic(). */
    @Override
    public OptionalDouble getPosition() {
        return m_encoder.getPositionRad();
    }

    /** Value is updated in Robot.robotPeriodic(). */
    @Override
    public OptionalDouble getVelocity() {
        return m_encoder.getRateRad_S();
    }

    /**
     * Compares robotPeriodic-updated measurements to the goal,
     * so you need to know when the goal was updated: is it for the
     * current Takt time, or the next step?
     */
    @Override
    public boolean atSetpoint() {
        OptionalDouble positionRad = getPosition();
        if (positionRad.isEmpty())
            return false;
        OptionalDouble velocityRad_S = getVelocity();
        if (velocityRad_S.isEmpty())
            return false;
        double positionError = MathUtil.angleModulus(m_goal.x() - positionRad.getAsDouble());
        double velocityError = m_goal.v() - velocityRad_S.getAsDouble();
        return Math.abs(positionError) < kPositionTolerance
                && Math.abs(velocityError) < kVelocityTolerance;
    }

    @Override
    public boolean profileDone() {
        return atGoal();
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
        return atSetpoint();
    }

    @Override
    public double getGoal() {
        return m_goal.x();
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    /** for this controller, the goal is the setpoint */
    @Override
    public Control100 getSetpoint() {
        return m_goal.control();
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_encoder.periodic();
    }

    @Override
    public void setStaticTorque(double feedForwardTorqueNm) {
    }

    @Override
    public void setProfile(Profile100 profile){
        // m_controller.setProfile(profile);
    }

}
