package org.team100.lib.profile.r1;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public class MockIncrementalProfile implements IncrementalProfile {
    Control100 result;
    double eta;
    int count = 0;

    @Override
    public Control100 calculate(double dt, Control100 initial, Model100 goal) {
        count++;
        return result;
    }

    @Override
    public MockIncrementalProfile scale(double s) {
        return this;
    }

    public double getMaxVelocity() {
        return 0;
    }

    @Override
    public double solve(double dt, Control100 i, Model100 g, double eta, double etaTolerance) {
        return 1.0;
    }

}
