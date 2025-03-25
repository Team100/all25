package org.team100.lib.profile;

import org.team100.lib.profile.jerk_limited.MotionProfile;
import org.team100.lib.profile.jerk_limited.MotionProfileGenerator;
import org.team100.lib.profile.jerk_limited.MotionState;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/** Adapter for Roadrunner jerk-limited profiles. */
public class JerkLimitedProfile100 implements SimpleProfile100 {

    private final double m_vel;
    private final double m_acc;
    private final double m_jerk;

    public JerkLimitedProfile100(double vel, double acc, double jerk) {
        m_vel = vel;
        m_acc = acc;
        m_jerk = jerk;
    }

    @Override
    public Control100 calculate(double dt, Model100 initial, Model100 goal) {
        MotionState start = new MotionState(initial.x(), initial.v());
        MotionState end = new MotionState(goal.x(), goal.v());
        // TODO: compute the profile once
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, end, m_vel, m_acc, m_jerk);
        MotionState s = profile.get(dt);
        return new Control100(s.getX(), s.getV(), s.getA());
    }

}
