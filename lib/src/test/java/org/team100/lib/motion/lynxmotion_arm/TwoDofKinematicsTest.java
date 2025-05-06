package org.team100.lib.motion.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class TwoDofKinematicsTest {

    private static final double kDelta = 0.001;

    @Test
    public void testf1() {
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        TwoDofKinematics.ArmAngles a = new TwoDofKinematics.ArmAngles(0, 0);
        Translation2d f = k.forward(a);
        assertEquals(2, f.getX(), kDelta);
        assertEquals(0, f.getY(), kDelta);
    }

    @Test
    public void testf2() {
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        TwoDofKinematics.ArmAngles a = new TwoDofKinematics.ArmAngles(0, Math.PI / 2);
        Translation2d f = k.forward(a);
        assertEquals(1, f.getX(), kDelta);
        assertEquals(1, f.getY(), kDelta);
    }

    @Test
    public void testf3() {
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        TwoDofKinematics.ArmAngles a = new TwoDofKinematics.ArmAngles(-Math.PI / 6, 2 * Math.PI / 3);
        Translation2d f = k.forward(a);
        assertEquals(Math.sqrt(3) / 2, f.getX(), kDelta);
        assertEquals(0.5, f.getY(), kDelta);
    }

    @Test
    public void testi1() {
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        Translation2d t = new Translation2d(2, 0);
        TwoDofKinematics.ArmAngles a = k.inverse(t);
        assertEquals(0, a.th1, kDelta);
        assertEquals(0, a.th2, kDelta);
    }

    @Test
    public void testi2() {
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        Translation2d t = new Translation2d(1, 1);
        TwoDofKinematics.ArmAngles a = k.inverse(t);
        assertEquals(0, a.th1, kDelta);
        assertEquals(Math.PI / 2, a.th2, kDelta);
    }

    @Test
    public void testi3() {
        TwoDofKinematics k = new TwoDofKinematics(1, 1);
        Translation2d t = new Translation2d(Math.sqrt(3) / 2, 0.5);
        TwoDofKinematics.ArmAngles a = k.inverse(t);
        assertEquals(-Math.PI / 6, a.th1, kDelta);
        assertEquals(2 * Math.PI / 3, a.th2, kDelta);
    }
}