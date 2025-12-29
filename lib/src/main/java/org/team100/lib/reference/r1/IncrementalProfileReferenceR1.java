package org.team100.lib.reference.r1;

import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.SetpointsR1Logger;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Extracts current and next references from an incremental profile.
 * 
 * Uses a profile supplier so it can be updated on the fly.
 */
public class IncrementalProfileReferenceR1 implements ProfileReferenceR1 {
    private final SetpointsR1Logger m_log_setpoints;
    private final BooleanLogger m_log_done;
    private final Supplier<IncrementalProfile> m_profile;
    private final double m_positionTolerance;
    private final double m_velocityTolerance;
    private Model100 m_goal;
    private double m_currentInstant;
    private SetpointsR1 m_currentSetpoint;

    public IncrementalProfileReferenceR1(
            LoggerFactory parent,
            Supplier<IncrementalProfile> profile,
            double positionTolerance,
            double velocityTolerance) {
        LoggerFactory log = parent.type(this);
        m_log_setpoints = log.setpointsR1Logger(Level.TRACE, "setpoints");
        m_log_done = log.booleanLogger(Level.TRACE, "done");
        m_profile = profile;
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    @Override
    public void setGoal(Model100 goal) {
        m_goal = goal;
    }

    @Override
    public void init(Model100 measurement) {
        m_currentInstant = Takt.get();
        m_currentSetpoint = advance(measurement.control());
    }

    @Override
    public SetpointsR1 get() {
        double t = Takt.get();
        if (t == m_currentInstant) {
            // Time hasn't passed since last time, so don't change anything.
            return m_currentSetpoint;
        }

        // Time has passed, make a new setpoint and return it.
        m_currentInstant = t;
        m_currentSetpoint = advance(m_currentSetpoint.next());
        m_log_setpoints.log(() -> m_currentSetpoint);
        return m_currentSetpoint;
    }

    @Override
    public boolean profileDone() {
        // the only way to tell if an incremental profile is done is to compare the goal
        // to the setpoint.
        boolean done = m_currentSetpoint.current().model().near(m_goal, m_positionTolerance, m_velocityTolerance);
        m_log_done.log(() -> done);
        return done;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////

    private SetpointsR1 advance(Control100 newCurrent) {
        if (m_goal == null)
            throw new IllegalStateException("goal must be set");
        Control100 next = m_profile.get().calculate(TimedRobot100.LOOP_PERIOD_S, newCurrent, m_goal);
        return new SetpointsR1(newCurrent, next);
    }
}
