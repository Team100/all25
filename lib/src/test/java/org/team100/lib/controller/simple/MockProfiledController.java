package org.team100.lib.controller.simple;

import org.team100.lib.state.Model100;

public class MockProfiledController implements ProfiledController {

    public Result result;

    @Override
    public void init(Model100 measurement) {

    }

    @Override
    public Result calculate(Model100 measurement, Model100 goal) {
        return result;
    }

    @Override
    public boolean profileDone() {
        return false;
    }

    @Override
    public Model100 getSetpoint() {
        return null;
    }

    @Override
    public boolean atSetpoint() {
        return false;
    }

    @Override
    public boolean atGoal(Model100 goal) {
        return false;
    }

    @Override
    public void close() {
        
    }

}
