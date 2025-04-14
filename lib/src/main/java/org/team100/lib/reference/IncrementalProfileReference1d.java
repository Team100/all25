package org.team100.lib.reference;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Takt;

/**
 * To extract current and next references from the incremental profile
 * controller.
 * 
 * Contains the goal, since our goals never change.
 */
public class IncrementalProfileReference1d {
    private final IncrementalProfile m_profile;
    protected Model100 m_goal;
    private double m_currentInstant;
    private Setpoints1d m_currentSetpoint;

    public IncrementalProfileReference1d(IncrementalProfile profile, Model100 goal) {
        m_profile = profile;
        m_goal = goal;
    }

    public void init(Model100 measurement) {
        m_currentInstant = Takt.get();
        m_currentSetpoint = advance(measurement.control());
    }

    public Setpoints1d get() {
        double t = Takt.get();
        // If time hasn't passed, don't change anything.
        if (t == m_currentInstant)
            return m_currentSetpoint;

        // Time has passed, make a new setpoint and return it.
        m_currentInstant = t;
        m_currentSetpoint = advance(m_currentSetpoint.next());
        return m_currentSetpoint;
    }

    private Setpoints1d advance(Control100 newCurrent) {
        if (m_goal == null)
            throw new IllegalStateException("goal must be set");
        Control100 next = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, newCurrent, m_goal);
        return new Setpoints1d(newCurrent, next);
    }

}
