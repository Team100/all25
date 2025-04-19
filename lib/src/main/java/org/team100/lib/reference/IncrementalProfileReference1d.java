package org.team100.lib.reference;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Takt;

/**
 * Extracts current and next references from an incremental profile.
 */
public class IncrementalProfileReference1d implements ProfileReference1d {
    private final IncrementalProfile m_profile;
    private final double m_positionTolerance;
    private final double m_velocityTolerance;
    private Model100 m_goal;
    private double m_currentInstant;
    private Setpoints1d m_currentSetpoint;

    public IncrementalProfileReference1d(
            IncrementalProfile profile,
            double positionTolerance,
            double velocityTolerance) {
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
    public Setpoints1d get() {
        double t = Takt.get();
        if (t == m_currentInstant) {
            // Time hasn't passed since last time, so don't change anything.
            return m_currentSetpoint;
        }

        // Time has passed, make a new setpoint and return it.
        m_currentInstant = t;
        m_currentSetpoint = advance(m_currentSetpoint.next());
        return m_currentSetpoint;
    }

    @Override
    public boolean profileDone() {
        // the only way to tell if an incremental profile is done is to compare the goal
        // to the setpoint.
        return m_currentSetpoint.current().model().near(m_goal, m_positionTolerance, m_velocityTolerance);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////

    private Setpoints1d advance(Control100 newCurrent) {
        if (m_goal == null)
            throw new IllegalStateException("goal must be set");
        Control100 next = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, newCurrent, m_goal);
        return new Setpoints1d(newCurrent, next);
    }
}
