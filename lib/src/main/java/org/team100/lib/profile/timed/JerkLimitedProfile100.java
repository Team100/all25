package org.team100.lib.profile.timed;

import org.team100.lib.profile.jerk_limited.MotionProfile;
import org.team100.lib.profile.jerk_limited.MotionProfileGenerator;
import org.team100.lib.profile.jerk_limited.MotionState;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

/** Adapter for Roadrunner jerk-limited profiles. */
public class JerkLimitedProfile100 implements TimedProfile {
    private static final boolean DEBUG = false;

    private final double vel;
    private final double acc;
    private final double jerk;
    private MotionProfile m_profile;

    /** You can specify independent limits for velocity, acceleration, and jerk. */
    public JerkLimitedProfile100(double vel, double acc, double jerk) {
        this.vel = vel;
        this.acc = acc;
        this.jerk = jerk;
    }

    @Override
    public void init(Model100 initial, Model100 goal) {
        if (DEBUG)
            Util.printf("INIT initial %s goal %s\n", initial, goal);
        MotionState start = new MotionState(initial.x(), initial.v());
        MotionState end = new MotionState(goal.x(), goal.v());
        m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, end, vel, acc, jerk);
    }

    @Override
    public Control100 sample(double timeS) {
        if (DEBUG)
            Util.printf("time %f\n", timeS);
        MotionState s = m_profile.get(timeS);
        return new Control100(s.getX(), s.getV(), s.getA());
    }

    @Override
    public double duration() {
        return m_profile.duration();
    }

}
