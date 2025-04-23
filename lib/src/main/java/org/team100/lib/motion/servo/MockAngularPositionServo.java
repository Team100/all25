package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.reference.Setpoints1d;

/**
 * This is just for tests and illustration.
 */
public class MockAngularPositionServo implements AngularPositionServo {

    @Override
    public void reset() {
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
    }

    @Override
    public void setPositionProfiled(double goalRad, double feedForwardTorqueNm) {
    }

    @Override
    public void setPositionDirect(Setpoints1d setpoint, double feedForwardTorqueNm) {
    }

    @Override
    public OptionalDouble getPosition() {
        return null;
    }

    @Override
    public boolean atSetpoint() {
        return false;
    }

    @Override
    public boolean profileDone() {
        return false;
    }

    @Override
    public boolean atGoal() {
        return false;
    }

    @Override
    public void stop() {
    }

    @Override
    public void close() {
    }

    @Override
    public void periodic() {
    }

}
