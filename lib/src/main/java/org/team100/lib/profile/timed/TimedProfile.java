package org.team100.lib.profile.timed;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * A timed profile is analogous to a trajectory: you precalculate something,
 * and then you sample it by time.
 */
public interface TimedProfile {

    /**
     * Initial is Control100 so we can make acceleration smooth between multiple
     * profiles.
     */
    void init(Control100 initial, Model100 goal);

    Control100 sample(double timeS);

    double duration();

}
