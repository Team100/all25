package org.team100.lib.motion.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motor.MockBareMotor;

public class LinearMechanismTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    /** Show that the limits have effect. */
    @Test
    void testLimits() {
        Feedforward100 ff = Feedforward100.makeSimple();
        MockBareMotor motor = new MockBareMotor(ff);
        MockIncrementalBareEncoder encoder = new MockIncrementalBareEncoder();
        double gearRatio = 1;
        double wheelDiameterM = 2;
        LinearMechanism mech = new LinearMechanism(motor, encoder, gearRatio, wheelDiameterM, 1, 2);

        // duty cycle limit observes the encoder
        // within bounds => ok
        encoder.position = 1.5;
        mech.setDutyCycle(1.0);
        assertEquals(1.0, motor.output, kDelta);
        // out of bounds => stop.
        encoder.position = 2.5;
        mech.setDutyCycle(1.0);
        assertEquals(0.0, motor.output, kDelta);

        // velocity limit observes the encoder
        // within bounds => ok
        encoder.position = 1.5;
        mech.setVelocity(1.0, 0, 0);
        assertEquals(1.0, motor.velocity, kDelta);
        // out of bounds => stop
        encoder.position = 2.5;
        mech.setVelocity(1.0, 0, 0);
        assertEquals(0.0, motor.velocity, kDelta);

        // positional limits filter the input
        // within bounds => ok
        mech.setPosition(1.5, 1.0, 0, 0);
        assertEquals(1.0, motor.velocity, kDelta);
        // out of bounds => stop
        mech.setPosition(2.5, 1.0, 0, 0);
        assertEquals(0.0, motor.velocity, kDelta);
    }

    /** Same cases as above, but unlimited */
    @Test
    void testUnlimited() {
        Feedforward100 ff = Feedforward100.makeSimple();
        MockBareMotor motor = new MockBareMotor(ff);
        MockIncrementalBareEncoder encoder = new MockIncrementalBareEncoder();
        double gearRatio = 1;
        double wheelDiameterM = 2;
        LinearMechanism mech = new LinearMechanism(
                motor, encoder, gearRatio, wheelDiameterM, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        // duty cycle limit observes the encoder
        // within bounds => ok
        encoder.position = 1.5;
        mech.setDutyCycle(1.0);
        assertEquals(1.0, motor.output, kDelta);
        // out of bounds => stop.
        encoder.position = 2.5;
        mech.setDutyCycle(1.0);
        assertEquals(1.0, motor.output, kDelta);

        // velocity limit observes the encoder
        // within bounds => ok
        encoder.position = 1.5;
        mech.setVelocity(1.0, 0, 0);
        assertEquals(1.0, motor.velocity, kDelta);
        // out of bounds => stop
        encoder.position = 2.5;
        mech.setVelocity(1.0, 0, 0);
        assertEquals(1.0, motor.velocity, kDelta);

        // positional limits filter the input
        // within bounds => ok
        mech.setPosition(1.5, 1.0, 0, 0);
        assertEquals(1.0, motor.velocity, kDelta);
        // out of bounds => stop
        mech.setPosition(2.5, 1.0, 0, 0);
        assertEquals(1.0, motor.velocity, kDelta);
    }

}
