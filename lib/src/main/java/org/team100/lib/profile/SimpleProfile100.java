package org.team100.lib.profile;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * One-dimensional profile, doesn't support coordination.
 */
public interface SimpleProfile100 {
    /**
     * Return the control for dt in the future.
     * 
     * Note order here, initial first, goal second.
     */
    Control100 calculate(double dt, Model100 initial, Model100 goal);

}
