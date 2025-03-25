package org.team100.lib.profile.timed;

import org.team100.lib.state.Control100;

/**
 * A timed profile is analogous to a trajectory: you precalculate something,
 * and then you sample it by time.
 */
public interface TimedProfile {

    Control100 sample(double timeS);

}
