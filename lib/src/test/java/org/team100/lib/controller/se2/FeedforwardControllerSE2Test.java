package org.team100.lib.controller.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.Control100;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelSE2;

public class FeedforwardControllerSE2Test {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testMotionless() {
        FeedforwardControllerSE2 c = new FeedforwardControllerSE2(logger, 0.01, 0.01, 0.01, 0.01);
        assertFalse(c.atReference());
        VelocitySE2 v = c.calculate(
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlSE2(
                        new Control100(0, 0),
                        new Control100(0, 0),
                        new Control100(0, 0)));
        assertEquals(0, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(0, v.theta(), DELTA);
        assertTrue(c.atReference());
    }

    @Test
    void testNotAtReference() {
        FeedforwardControllerSE2 c = new FeedforwardControllerSE2(logger, 0.01, 0.01, 0.01, 0.01);
        assertFalse(c.atReference());
        VelocitySE2 v = c.calculate(
                new ModelSE2(
                        new Model100(1, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlSE2(
                        new Control100(0, 0),
                        new Control100(0, 0),
                        new Control100(0, 0)));
        assertEquals(0, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(0, v.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testFeedforward() {
        FeedforwardControllerSE2 c = new FeedforwardControllerSE2(logger, 0.01, 0.01, 0.01, 0.01);
        assertFalse(c.atReference());
        VelocitySE2 v = c.calculate(
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlSE2(
                        new Control100(0, 1),
                        new Control100(0, 0),
                        new Control100(0, 0)));
        assertEquals(1, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(0, v.theta(), DELTA);
        assertTrue(c.atReference());
    }
}
