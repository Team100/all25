package org.team100.lib.profile.incremental;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/** Always returns the initial state. */
public class NullProfile implements Profile100 {

    @Override
    public Control100 calculate(double dt, Model100 initial, Model100 goal) {
        return initial.control();
    }

    @Override
    public ResultWithETA calculateWithETA(double dt, Model100 initial, Model100 goal) {
        return new ResultWithETA(initial.control(), 0);
    }

    @Override
    public NullProfile scale(double s) {
        return this;
    }

    @Override
    public double getMaxVelocity() {
        return 0;
    }

    @Override
    public double solve(double dt, Model100 i, Model100 g, double eta, double etaTolerance) {
        return 1.0;
    }

}
