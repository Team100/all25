package org.team100.lib.profile;

import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Switch from one timed profile to another at a specific time.
 * 
 * To use this effectively, you'll have to experiment a bit with the switching
 * time. See DualProfile3Test.
 * 
 * TODO: make the choice of switching time automatic, at least for rest-to-rest
 * profiles, which is mostly what we do.
 */
public class DualProfile3 implements TimedProfile {
    private final TimedProfile m_start;
    private final TimedProfile m_end;
    private final double m_switchS;
    private Model100 m_goal;
    private boolean m_switched;

    public DualProfile3(TimedProfile start, TimedProfile end, double switchS) {
        m_start = start;
        m_end = end;
        m_switchS = switchS;
    }

    @Override
    public void init(Control100 initial, Model100 goal) {
        m_goal = goal;
        m_start.init(initial, goal);
        m_switched = false;
    }

    @Override
    public Control100 sample(double timeS) {
        if (timeS < m_switchS) {
            return m_start.sample(timeS);
        }
        if (!m_switched) {
            // start the second profile
            Control100 initial = m_start.sample(timeS);
            m_end.init(initial, m_goal);
            m_switched = true;
        }
        return m_end.sample(timeS - m_switchS);
    }

    @Override
    public double duration() {
        // TODO: this is super wrong
        return m_switchS + m_end.duration();
    }

}
