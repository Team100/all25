package org.team100.lib.reference;

import org.team100.lib.state.Model100;

public class MockProfileReference1d implements ProfileReference1d {

    @Override
    public void setGoal(Model100 goal) {
        //
    }

    @Override
    public void init(Model100 measurement) {
        //
    }

    @Override
    public Setpoints1d get() {
        return null;
    }

    @Override
    public boolean profileDone() {
        return true;
    }
    
}
