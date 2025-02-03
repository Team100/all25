package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.util.Util;

public class OutboardAngularPositionServoTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final boolean kActuallyPrint = false;

    @Test
    void testOutboard() {
        final MockBareMotor motor = new MockBareMotor();
        final RotaryMechanism mech = new SimpleRotaryMechanism(
                logger,
                motor,
                new MockIncrementalBareEncoder(),
                1);
        final MockRotaryPositionSensor externalEncoder = new MockRotaryPositionSensor();
        final CombinedEncoder combinedEncoder = new CombinedEncoder(logger, externalEncoder, mech);
        final Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);

        final OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                logger,
                mech,
                combinedEncoder,
                profile);
        servo.reset();
        // it moves slowly
        servo.setPosition(1, 0);
        assertEquals(2e-4, motor.position, 1e-4);
        servo.setPosition(1, 0);
        // assertEquals(8e-4, motor.position, 1e-4);
        servo.setPosition(1, 0);
        assertEquals(0.002, motor.position, kDelta);
        for (int i = 0; i < 100; ++i) {
            // run it for awhile
            servo.setPosition(1, 0);
            if (kActuallyPrint)
                Util.printf("i: %d position: %5.3f\n", i, motor.position);
        }
        assertEquals(1, motor.position, kDelta);
    }
}
