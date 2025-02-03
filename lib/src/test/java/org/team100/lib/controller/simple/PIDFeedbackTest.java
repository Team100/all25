package org.team100.lib.controller.simple;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.Model100;

public class PIDFeedbackTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    public void testCalculate() {
        Feedback100 c = new PIDFeedback(logger, 1, 0, 0, false, 0.05, 1);

        // zero error, zero feedback
        double u = c.calculate(new Model100(), new Model100());
        assertEquals(0, u, kDelta);

        // position error -> u
        u = c.calculate(new Model100(0, 0), new Model100(1, 0));
        assertEquals(1, u, kDelta);

        // this controller ignores velocity
        u = c.calculate(new Model100(0, 0), new Model100(0, 1));
        assertEquals(0, u, kDelta);
    }
}
