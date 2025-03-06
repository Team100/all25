package org.team100.lib.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motor.MockBareMotor;

class CombinedEncoderTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testZeroing() {
        MockBareMotor motor = new MockBareMotor(Feedforward100.makeSimple());

        // this is the "correct" value
        MockRotaryPositionSensor e1 = new MockRotaryPositionSensor();
        e1.angle = 1;

        // this value is the "incorrect" value, should be overwritten by the combined
        // encoder constructor.
        MockIncrementalBareEncoder e2 = new MockIncrementalBareEncoder();
        e2.position = 0;

        RotaryMechanism m = new SimpleRotaryMechanism(logger, motor, e2, 1.0);
        CombinedEncoder c = new CombinedEncoder(logger, e1, m, true);
        c.sync();
        // the combined encoder reads the correct value
        assertEquals(1.0, c.getPositionRad().getAsDouble(), kDelta);

        // and the secondary encoder has been "fixed"
        assertEquals(1.0, e2.position, kDelta);
    }
}
