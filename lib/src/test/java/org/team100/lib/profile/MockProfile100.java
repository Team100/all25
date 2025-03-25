package org.team100.lib.profile;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public class MockProfile100 implements Profile100 {
    Control100 result;
    double eta;
    int count = 0;

    @Override
    public Control100 calculate(double dt, Model100 initial, Model100 goal) {
        count++;
        return result;
    }

    @Override
    public ResultWithETA calculateWithETA(double dt, Model100 initial, Model100 goal) {
        return new ResultWithETA(result, eta);
    }

    @Override
    public MockProfile100 scale(double s) {
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
