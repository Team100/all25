package org.team100.lib.controller.simple;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public class PIDControllerPosWPITest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    public void testCalculate() {
        PIDControllerPosWPI c = new PIDControllerPosWPI(logger, 1, 0, 0, false);

        // measurement == goal: control is the same
        Control100 u = c.calculate(new Model100(), new Model100());
        assertEquals(0, u.x(), kDelta);
        assertEquals(0, u.v(), kDelta);
        assertEquals(0, u.a(), kDelta);

        // goal x=1, so error=1, so new p=1 (!)
        u = c.calculate(new Model100(0, 0), new Model100(1, 0));
        assertEquals(1, u.x(), kDelta);
        assertEquals(50, u.v(), kDelta);
        assertEquals(2500, u.a(), kDelta);

        // this controller ignores velocity
        u = c.calculate(new Model100(0, 0), new Model100(0, 1));
        assertEquals(0, u.x(), kDelta);
        assertEquals(0, u.v(), kDelta);
        assertEquals(0, u.a(), kDelta);
    }
}
