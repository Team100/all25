package org.team100.lib.profile.roadrunner;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class DynamicProfileGeneratorTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.001;

    @Test
    void testGenerateMotionProfile() {
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(5, 0, 0, 0);
        VelocityConstraint v = new VelocityConstraint() {

            @Override
            public double get(double s) {
                return 1;
            }

        };
        AccelerationConstraint a = new AccelerationConstraint() {

            @Override
            public double get(double s) {
                return 1;
            }

        };
        double resolution = 1;
        MotionProfile p = DynamicProfileGenerator.generateMotionProfile(start, goal, v, a, resolution);

        assertEquals(7, p.getSegments().size());
        assertEquals(0, p.get(0).getX(), kDelta);
        assertEquals(0.5, p.get(1).getX(), kDelta);
        assertEquals(1.5, p.get(2).getX(), kDelta);
        assertEquals(2.5, p.get(3).getX(), kDelta);
        assertEquals(3.5, p.get(4).getX(), kDelta);
        assertEquals(4.5, p.get(5).getX(), kDelta);
        assertEquals(5.0, p.get(6).getX(), kDelta);

        assertEquals(6.0, p.duration(), kDelta);

        MotionState s0 = p.get(0);
        assertEquals(0, s0.getV(), kDelta);

        MotionProfile p1 = p.append(p);
        assertEquals(12, p1.duration(), kDelta);

        MotionState s1 = p.get(1);
        assertEquals(1, s1.getV(), kDelta);
    }

}
