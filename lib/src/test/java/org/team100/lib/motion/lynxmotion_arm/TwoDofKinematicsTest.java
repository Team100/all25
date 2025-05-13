package org.team100.lib.motion.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.lynxmotion_arm.TwoDofKinematics.TwoDofArmConfig;
import org.team100.lib.motion.lynxmotion_arm.TwoDofKinematics.TwoDofArmPosition;

import edu.wpi.first.math.geometry.Translation2d;

public class TwoDofKinematicsTest {
    private static final double kDelta = 0.001;

    @Test
    void testf1() {
        // stretched along x
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(1, 0), new Translation2d(2, 0));
        TwoDofArmConfig q = new TwoDofArmConfig(0, 0);
        verify(k, p, q);
    }

    @Test
    void testf2() {
        // up and then out
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(0, 1), new Translation2d(1, 1));
        TwoDofArmConfig q = new TwoDofArmConfig(Math.PI / 2, -1 * Math.PI / 2);
        verify(k, p, q);
    }

    @Test
    void testf3() {
        // equilateral triangle, first link up
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(0, 1), new Translation2d(Math.sqrt(3) / 2, 0.5));
        TwoDofArmConfig q = new TwoDofArmConfig(Math.PI / 2, -2 * Math.PI / 3);
        verify(k, p, q);
    }

    @Test
    void test4() {
        // vertical equilateral triangle
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(-Math.sqrt(3) / 2, 0.5), new Translation2d(0, 1));
        TwoDofArmConfig q = new TwoDofArmConfig(5 * Math.PI / 6, -2 * Math.PI / 3);
        verify(k, p, q);
    }

    @Test
    void test5() {
        // behind
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(-1, 0), new Translation2d(-1, 1));
        TwoDofArmConfig q = new TwoDofArmConfig(Math.PI, -Math.PI / 2);
        verify(k, p, q);
    }

    void verify(TwoDofKinematics k, TwoDofArmPosition p, TwoDofArmConfig q) {
        verifyFwd(p, k.forward(q));
        verifyInv(q, k.inverse(p.p2()));
    }

    void verifyFwd(TwoDofArmPosition expected, TwoDofArmPosition actual) {
        assertEquals(expected.p1(), actual.p1(), "fwd p1");
        assertEquals(expected.p2(), actual.p2(), "fwd p2");
    }

    void verifyInv(TwoDofArmConfig expected, TwoDofArmConfig actual) {
        assertEquals(expected.q1(), actual.q1(), kDelta, "inv q1");
        assertEquals(expected.q2(), actual.q2(), kDelta, "inv q2");
    }
}