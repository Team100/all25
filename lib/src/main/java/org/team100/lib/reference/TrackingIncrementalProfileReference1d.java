package org.team100.lib.reference;

import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.state.Model100;

/**
 * Allows the goal to be updated.
 */
public class TrackingIncrementalProfileReference1d extends IncrementalProfileReference1d {
    public TrackingIncrementalProfileReference1d(
            IncrementalProfile profile,
            double positionTolerance,
            double velocityTolerance) {
        super(profile, null, positionTolerance, velocityTolerance);
    }

    public void setGoal(Model100 goal) {
        m_goal = goal;
    }
}
