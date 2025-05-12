package org.team100.lib.motion.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class TwoDofKinematicsTest {
    private static final double kDelta = 0.001;

    @Test
    public void testf1() {
        // stretched along x
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        {
            TwoDofKinematics.TwoDofArmConfig a = new TwoDofKinematics.TwoDofArmConfig(0, 0);
            Translation2d f = k.forward(a);
            assertEquals(2, f.getX(), kDelta);
            assertEquals(0, f.getY(), kDelta);
        }
        {
            Translation2d t = new Translation2d(2, 0);
            TwoDofKinematics.TwoDofArmConfig a = k.inverse(t);
            assertEquals(0, a.q1(), kDelta);
            assertEquals(0, a.q2(), kDelta);
        }
    }

    @Test
    public void testf2() {
        // up and then out
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        {
            TwoDofKinematics.TwoDofArmConfig a = new TwoDofKinematics.TwoDofArmConfig(0, Math.PI / 2);
            Translation2d f = k.forward(a);
            assertEquals(1, f.getX(), kDelta);
            assertEquals(1, f.getY(), kDelta);
        }
        {
            Translation2d t = new Translation2d(1, 1);
            TwoDofKinematics.TwoDofArmConfig a = k.inverse(t);
            assertEquals(Math.PI / 2, a.q1(), kDelta);
            assertEquals(-1 * Math.PI / 2, a.q2(), kDelta);
        }
    }

    @Test
    public void testf3() {
        // equilateral triangle, first link up
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        {
            TwoDofKinematics.TwoDofArmConfig a = new TwoDofKinematics.TwoDofArmConfig(Math.PI / 2, -2 * Math.PI / 3);
            Translation2d f = k.forward(a);
            assertEquals(Math.sqrt(3) / 2, f.getX(), kDelta);
            assertEquals(0.5, f.getY(), kDelta);
        }
        {
            Translation2d t = new Translation2d(Math.sqrt(3) / 2, 0.5);
            TwoDofKinematics.TwoDofArmConfig a = k.inverse(t);
            assertEquals(Math.PI / 2, a.q1(), kDelta);
            assertEquals(-2 * Math.PI / 3, a.q2(), kDelta);
        }
    }
}