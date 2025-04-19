package org.team100.lib.reference;

import org.team100.lib.state.Model100;

/**
 * Provides current and next references for servos.
 * 
 * Goal must be set prior to initialization.
 */
public interface ProfileReference1d {

    void setGoal(Model100 goal);

    void init(Model100 measurement);

    Setpoints1d get();

    boolean profileDone();
}
