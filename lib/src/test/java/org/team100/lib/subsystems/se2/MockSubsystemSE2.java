package org.team100.lib.subsystems.se2;

import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.state.ModelSE2;

public class MockSubsystemSE2 implements VelocitySubsystemSE2 {
    public VelocitySE2 m_setpoint;
    public VelocitySE2 m_recentSetpoint;
    public ModelSE2 m_state;

    public MockSubsystemSE2(ModelSE2 initial) {
        m_state = initial;
    }

    @Override
    public ModelSE2 getState() {
        return m_state;
    }

    @Override
    public void stop() {
        // do nothing
    }

    @Override
    public void setVelocity(VelocitySE2 setpoint) {
        m_setpoint = setpoint;
        m_recentSetpoint = setpoint;
    }

}
