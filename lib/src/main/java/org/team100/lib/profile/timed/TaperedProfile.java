package org.team100.lib.profile.timed;

import org.team100.lib.profile.roadrunner.DynamicProfileGenerator;
import org.team100.lib.profile.roadrunner.MotionProfile;
import org.team100.lib.profile.roadrunner.MotionState;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Adapter for Roadrunner dynamic profiles.
 * 
 * The use-case is soft-landing, so the acceleration constraint tapers to zero
 * at the goal.
 * 
 * If the taper is linear, then the goal is never reached; we use the cube root
 * to obtain
 * roughly linear tapering with time, i.e. constant jerk
 */
public class TaperedProfile implements TimedProfile {
    private static final boolean DEBUG = false;

    private final double vel;
    private final double acc;
    private final double taper;
    private final double resolution;
    private MotionProfile m_profile;

    /**
     * Taper the acceleration near the goal kinda coarsely, using
     * constant-acceleration segments.
     * 
     * @param vel
     * @param acc
     * @param jerk       I kinda guessed what to do with this parameter, so it's
     *                   only like very roughly the jerk. The actual jerk is pretty
     *                   high between segments.
     * @param resolution how many constant-accel segments to use; note these are
     *                   segmented by *distance* so the last few are larger in time.
     *                   Higher resolution takes a little longer to calculate.
     */
    public TaperedProfile(double vel, double acc, double jerk, double resolution) {
        this.vel = vel;
        this.acc = acc;
        // i have no idea if this is right; it does seem to fit the data.
        this.taper = 1.8 * Math.pow(jerk, 2.0 / 3);
        this.resolution = resolution;
    }

    @Override
    public void init(Control100 initial, Model100 goal) {
        MotionState start = new MotionState(initial.x(), initial.v(), initial.a(), 0);
        MotionState end = new MotionState(goal.x(), goal.v(), 0, 0);
        m_profile = DynamicProfileGenerator.generateMotionProfile(
                start, end, (s) -> vel, (s) -> {
                    double togo = goal.x() - s;
                    double taperedAcc = Math.pow(Math.abs(togo), 1.0 / 3) * taper;
                    return Math.min(acc, taperedAcc);
                }, resolution);
    }

    @Override
    public Control100 sample(double timeS) {
        MotionState s = m_profile.get(timeS);
        return new Control100(s.x(), s.v(), s.a());
    }

    @Override
    public double duration() {
        return m_profile.duration();
    }

}
