package org.team100.lib.profile;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public interface Profile100 {

    public static record ResultWithETA(Control100 state, double etaS) {
    }

    /**
     * Return the control for dt in the future.
     * 
     * Note order here, initial first, goal second.
     */
    Control100 calculate(double dt, Model100 initial, Model100 goal);

    /**
     * Return the control for dt in the future.
     * 
     * Note order here, initial first, goal second.
     */
    ResultWithETA calculateWithETA(double dt, Model100 initial, Model100 goal);

    /**
     * Return a new profile with acceleration scaled by s.
     */
    Profile100 scale(double s);

}