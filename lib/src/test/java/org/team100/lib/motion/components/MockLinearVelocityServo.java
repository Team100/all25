package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.motion.servo.LinearVelocityServo;

public class MockLinearVelocityServo implements LinearVelocityServo {

    double m_setpoint;

    @Override
    public void reset() {
        //
    }

    @Override
    public void setVelocityM_S(double setpoint) {
        m_setpoint = setpoint;
    }

    @Override
    public void setVelocity(double setpoint, double setpoint_2) {
        m_setpoint = setpoint;
    }

    @Override
    public OptionalDouble getVelocity() {
        return OptionalDouble.of(m_setpoint);
    }

    @Override
    public void stop() {
        //
    }

    @Override
    public OptionalDouble getDistance() {
        throw new UnsupportedOperationException("Unimplemented method 'getDistance'");
    }

    @Override
    public void periodic() {
        //
    }

}
