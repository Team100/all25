package org.team100.lib.profile.r1;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/** Always returns the initial state. */
public class NullProfile implements IncrementalProfile {

    @Override
    public Control100 calculate(double dt, Control100 initial, Model100 goal) {
        return initial;
    }

    @Override
    public NullProfile scale(double s) {
        return this;
    }

    public double getMaxVelocity() {
        return 0;
    }
}
