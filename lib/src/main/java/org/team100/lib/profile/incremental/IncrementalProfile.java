package org.team100.lib.profile.incremental;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * This profile takes incremental steps from the setpoint towards the goal.
 * 
 * One-dimensional, doesn't support coordination.
 */
public interface IncrementalProfile {
    /**
     * Return the control for dt in the future. The setpoint is a Control100 so that
     * we can regulate jerk.
     */
    Control100 calculate(double dt, Control100 setpoint, Model100 goal);

}
