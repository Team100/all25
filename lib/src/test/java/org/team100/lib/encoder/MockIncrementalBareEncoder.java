package org.team100.lib.encoder;

import java.util.OptionalDouble;

/** Contains no logic. */
public class MockIncrementalBareEncoder implements IncrementalBareEncoder {
    public double position = 0;
    public double velocity = 0;

    @Override
    public OptionalDouble getVelocityRad_S() {
        return OptionalDouble.of(velocity);
    }

    @Override
    public OptionalDouble getPositionRad() {
        return OptionalDouble.of(position);
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void setEncoderPositionRad(double motorPositionRad) {
        position = motorPositionRad;
    }

    @Override
    public void periodic() {
        //
    }

    @Override
    public double getPositionBlockingRad() {
        return getPositionRad().getAsDouble();
    }

}
