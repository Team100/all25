package org.team100.lib.profile.roadrunner;

import static org.junit.jupiter.api.Assertions.assertEquals;

// passes uncommented
// import com.acmerobotics.roadrunner.profile.MotionProfile;
// import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
// import com.acmerobotics.roadrunner.profile.MotionState;
// import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
// import com.acmerobotics.roadrunner.profile.VelocityConstraint;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

public class JerkLimitedProfileGeneratorTest {
    private static final boolean DEBUG = true;
    private static final double kDelta = 0.001;

    /**
     * see
     * https://docs.google.com/spreadsheets/d/19WbkNaxcRGHwYwLH1pu9ER3qxZrsYqDlZTdV-cmOM0I
     * 
     */
    @Test
    void testSample() {
        // an example from 0 to 1, with constraints on derivatives at the ends and along
        // the path.
        // see Spline1dTest.testSample()
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(1, 0, 0, 0);
        double maxVel = 2;
        double maxAccel = 6;
        double maxJerk = 20;
        boolean overshoot = false;
        MotionProfile p = JerkLimitedProfileGenerator.generateMotionProfile(start, goal, maxVel, maxAccel,
                maxJerk,
                overshoot);
        for (double t = 0; t < p.duration(); t += 0.01) {
            MotionState state = p.get(t);
            if (DEBUG) {
                double x = state.x();
                double v = state.v();
                double a = state.a();
                double j = state.j();
                Util.printf("%8.3f %8.3f %8.3f %8.3f %8.3f\n",
                        t, x, v, a, j);
            }
        }
    }

    @Test
    void testMovingEntry() {
        // an example from 0 to 1, with constraints on derivatives at the ends and along
        // the path.
        // see Spline1dTest.testSample()
        MotionState start = new MotionState(0, 1, 0, 0);
        MotionState goal = new MotionState(1, 0, 0, 0);
        double maxVel = 1;
        double maxAccel = 0.1;
        double maxJerk = 100;
        boolean overshoot = false;
        MotionProfile p = JerkLimitedProfileGenerator.generateMotionProfile(start, goal, maxVel, maxAccel,
                maxJerk,
                overshoot);
        for (double t = 0; t < p.duration(); t += 0.01) {
            MotionState state = p.get(t);
            if (DEBUG) {
                double x = state.x();
                double v = state.v();
                double a = state.a();
                double j = state.j();
                Util.printf("%8.3f %8.3f %8.3f %8.3f %8.3f\n",
                        t, x, v, a, j);
            }
        }
    }

    @Test
    void testGenerateSimpleMotionProfile() {
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(10, 0, 0, 0);
        double maxVel = 1;
        double maxAccel = 1;
        double maxJerk = 1;
        boolean overshoot = false;
        MotionProfile p = JerkLimitedProfileGenerator.generateMotionProfile(start, goal, maxVel, maxAccel,
                maxJerk,
                overshoot);

        assertEquals(12, p.duration(), kDelta);

        MotionState s0 = p.get(0);
        assertEquals(0, s0.v(), kDelta);

        MotionProfile p1 = p.append(p);
        assertEquals(24, p1.duration(), kDelta);

        MotionState s1 = p.get(1);
        assertEquals(0.5, s1.v(), kDelta);
    }

    @Test
    public void testSampleCount() {
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(5, 0, 0, 0);
        double resolution = 1;
        double length = goal.x() - start.x();
        int samples = Math.max(2, (int) Math.ceil(length / resolution));
        assertEquals(5, samples);

    }


    @Test
    void testGenerateAccelProfile() {
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionProfile p = JerkLimitedProfileGenerator.generateAccelProfile(start, 2, 6, 20);
        // end-state accel is zero
        assertEquals(0.000, p.get(p.duration()).a(), 0.001);
        for (double t = 0; t < p.duration(); t += 0.01) {
            MotionState state = p.get(t);
            if (DEBUG) {
                double x = state.x();
                double v = state.v();
                double a = state.a();
                double j = state.j();
                Util.printf("%8.3f %8.3f %8.3f %8.3f %8.3f\n",
                        t, x, v, a, j);
            }
        }
    }
}