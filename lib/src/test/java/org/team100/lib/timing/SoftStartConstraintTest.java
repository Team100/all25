package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.timing.TimingConstraint.MinMaxAcceleration;

import edu.wpi.first.math.geometry.Pose2d;

public class SoftStartConstraintTest {
    @Test
    void testSlow() {
        SoftStartConstraint c = new SoftStartConstraint();
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(1, 0, 0), 0, 0);
        MinMaxAcceleration m = c.getMinMaxAcceleration(p, 0.05);
        assertEquals(Double.NEGATIVE_INFINITY, m.getMinAccel(), 0.001);
        assertEquals(0.5, m.getMaxAccel(), 0.001);
    }

    @Test
    void testFast() {
        SoftStartConstraint c = new SoftStartConstraint();
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(1, 0, 0), 0, 0);
        MinMaxAcceleration m = c.getMinMaxAcceleration(p, 1);
        assertEquals(Double.NEGATIVE_INFINITY, m.getMinAccel(), 0.001);
        assertEquals(Double.POSITIVE_INFINITY, m.getMaxAccel(), 0.001);
    }
}
