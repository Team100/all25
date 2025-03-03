package org.team100.lib.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motor.SimulatedBareMotor;

public class SimulatedBareEncoderTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSetAndReset() {
        // changing the encoder position should not produce a velocity signal.

        SimulatedBareMotor motor = new SimulatedBareMotor(log, 100);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        assertEquals(0, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(0, motor.getPositionRad(), kDelta);
        assertEquals(0, motor.getVelocityRad_S(), kDelta);

        encoder.setEncoderPositionRad(1);

        assertEquals(1, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(1, motor.getPositionRad(), kDelta);
        assertEquals(0, motor.getVelocityRad_S(), kDelta);

        encoder.reset();

        assertEquals(0, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(0, motor.getPositionRad(), kDelta);
        assertEquals(0, motor.getVelocityRad_S(), kDelta);
    }

}
