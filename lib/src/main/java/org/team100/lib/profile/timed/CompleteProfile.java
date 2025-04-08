package org.team100.lib.profile.timed;

import org.team100.lib.profile.jerk_limited.MotionProfile;
import org.team100.lib.profile.jerk_limited.MotionState;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Uses Roadrunner dynamic profile to implement all the things we want from a
 * motion profile for mechanisms:
 * 
 * * current limit
 * * back-EMF limit
 * * jerk limit tapering
 */
public class CompleteProfile implements TimedProfile {
    private MotionProfile m_profile;

    @Override
    public void init(Control100 initial, Model100 goal) {
        MotionState start = new MotionState(initial.x(), initial.v(), initial.a());
        MotionState end = new MotionState(goal.x(), goal.v());
    }

    @Override
    public Control100 sample(double timeS) {
        MotionState s = m_profile.get(timeS);
        return new Control100(s.getX(), s.getV(), s.getA());
    }

    @Override
    public double duration() {
        return m_profile.duration();
    }

}
