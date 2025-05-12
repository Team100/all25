package org.team100.lib.motion.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.lynxmotion_arm.TwoDofKinematics.TwoDofArmConfig;

import edu.wpi.first.math.geometry.Translation2d;

public class TwoDofKinematicsTest {
    private static final double kDelta = 0.001;

    @Test
    public void testf1() {
        // stretched along x
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        Translation2d t = new Translation2d(2, 0);
        TwoDofArmConfig q = new TwoDofArmConfig(0, 0);
        verify(k, t, q);
    }

    @Test
    public void testf2() {
        // up and then out
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        Translation2d t = new Translation2d(1, 1);
        TwoDofArmConfig q = new TwoDofArmConfig(Math.PI / 2, -1 * Math.PI / 2);
        verify(k, t, q);
    }

    @Test
    public void testf3() {
        // equilateral triangle, first link up
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        Translation2d t = new Translation2d(Math.sqrt(3) / 2, 0.5);
        TwoDofArmConfig q = new TwoDofArmConfig(Math.PI / 2, -2 * Math.PI / 3);
        verify(k, t, q);
    }

    @Test
    public void test4() {
        // vertical equilateral triangle
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        Translation2d t = new Translation2d(0, 1);
        TwoDofArmConfig q = new TwoDofArmConfig(5 * Math.PI / 6, -2 * Math.PI / 3);
        verify(k, t, q);
    }

    @Test
    public void test5() {
        // behind
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        Translation2d t = new Translation2d(-1, 1);
        TwoDofArmConfig q = new TwoDofArmConfig(Math.PI, - Math.PI / 2);
        verify(k, t, q);
    }

    void verify(TwoDofKinematics k, Translation2d p, TwoDofArmConfig q) {
        verifyFwd(p, k.forward(q));
        verifyInv(q, k.inverse(p));
    }

    void verifyFwd(Translation2d expected, Translation2d actual) {
        assertEquals(expected.getX(), actual.getX(), kDelta, "fwd x");
        assertEquals(expected.getY(), actual.getY(), kDelta, "fwd y");
    }

    void verifyInv(TwoDofArmConfig expected, TwoDofArmConfig actual) {
        assertEquals(expected.q1(), actual.q1(), kDelta, "inv q1");
        assertEquals(expected.q2(), actual.q2(), kDelta, "inv q2");
    }
}