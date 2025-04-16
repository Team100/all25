package org.team100.lib.encoder;

import java.util.OptionalDouble;

/** Contains no logic. */
public class MockRotaryPositionSensor implements RotaryPositionSensor {
    public double angle = 0;
    public double rate = 0;

    @Override
    public OptionalDouble getPositionRad() {
        return OptionalDouble.of(angle);
    }

    @Override
    public OptionalDouble getVelocityRad_S() {
        return OptionalDouble.of(rate);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        //
    }

}
