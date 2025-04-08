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
    private final boolean overshoot;
    private MotionProfile m_profile;

    /**
     * You can specify independent limits for velocity, acceleration, and jerk.
     * 
     * @param overshoot if a single profile is impossible, true means to use one
     *                  that overshoots the goal, and a second one that returns to
     *                  it from the other side. false means to violate the
     *                  constraints.  you should probably say "true" here.
     */
    public JerkLimitedProfile100(double vel, double acc, double jerk, boolean overshoot) {
        this.vel = vel;
        this.acc = acc;
        this.jerk = jerk;
        this.overshoot = overshoot;
    }

    @Override
    public void init(Control100 initial, Model100 goal) {
        // if (Logging.instance().getLevel().admit(Level.TRACE))
        // Util.printf("INIT initial %s goal %s\n", initial, goal);
        MotionState start = new MotionState(initial.x(), initial.v(), initial.a());
        MotionState end = new MotionState(goal.x(), goal.v());
        // "true" below means "overshoot rather than violating constraints"
        m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, end, vel, acc, jerk, overshoot);
        if (DEBUG)
            Util.printf("profile %s\n", m_profile);
    }

    @Override
    public Control100 sample(double timeS) {
        MotionState s = m_profile.get(timeS);
        if (DEBUG)
            Util.printf("time %f x %f v %f a %f\n", timeS, s.getX(), s.getV(), s.getA());
        return new Control100(s.getX(), s.getV(), s.getA());
    }

    @Override
    public double duration() {
        return m_profile.duration();
    }

}
