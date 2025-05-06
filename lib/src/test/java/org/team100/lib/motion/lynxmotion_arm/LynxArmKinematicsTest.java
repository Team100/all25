package org.team100.lib.motion.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;

public class LynxArmKinematicsTest {

    private static final double kDelta = 0.001;

    // All three segments pointing up
    @Test
    void testf1() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 1, 1, 1);
        LynxArmAngles a = factory.fromRad(0, 0, 0, 0, 0, 0);
        Translation3d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(0, t.getY(), kDelta);
        assertEquals(3, t.getZ(), kDelta);
    }

    @Test
    void testf1_5() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 0.5, 0.5, 0.5);
        LynxArmAngles a = factory.fromRad(0, 0, 0, 0, 0, 0);
        Translation3d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(0, t.getY(), kDelta);
        assertEquals(1.5, t.getZ(), kDelta);
    }

    @Test
    void testi1() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 1, 1, 1);
        Translation3d t = new Translation3d(0, 0, 3);
        LynxArmAngles a = k.inverse(t, Math.PI / 2, 0, 0);
        assertEquals(0, a.swingRad(), kDelta);
        assertEquals(0, a.boomRad(), kDelta);
        assertEquals(0, a.stickRad(), kDelta);
        assertEquals(0, a.wristRad(), kDelta);
        assertEquals(0, a.twist, kDelta);
        assertEquals(0, a.grip, kDelta);
    }

    @Test
    void testi1_5() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 0.5, 0.5, 0.5);
        Translation3d t = new Translation3d(0, 0, 1.5);
        LynxArmAngles a = k.inverse(t, Math.PI / 2, 0, 0);
        assertEquals(0, a.swingRad(), kDelta);
        assertEquals(0, a.boomRad(), kDelta);
        assertEquals(0, a.stickRad(), kDelta);
        assertEquals(0, a.wristRad(), kDelta);
        assertEquals(0, a.twist, kDelta);
        assertEquals(0, a.grip, kDelta);
    }

    // bending forward at the elbow
    @Test
    void testf2() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 1, 1, 1);
        LynxArmAngles a = new LynxArmAngles.Factory().fromRad(0, 0, Math.PI / 2, 0, 0, 0);
        Translation3d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(2, t.getY(), kDelta);
        assertEquals(1, t.getZ(), kDelta);
    }

    @Test
    void testi2() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 1, 1, 1);
        Translation3d t = new Translation3d(0, 2, 1);
        LynxArmAngles a = k.inverse(t, 0, 0, 0);
        assertEquals(0, a.swingRad(), kDelta);
        assertEquals(0, a.boomRad(), kDelta);
        assertEquals(Math.PI / 2, a.stickRad(), kDelta);
        assertEquals(0, a.wristRad(), kDelta);
        assertEquals(0, a.twist, kDelta);
        assertEquals(0, a.grip, kDelta);
    }

    // wrist on the floor, wrist ahead
    @Test
    void testf3() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 1, 1, 1);
        LynxArmAngles a = new LynxArmAngles.Factory().fromRad(0, Math.PI / 6, 2 * Math.PI / 3, -Math.PI / 3, 0, 0);
        assertEquals(0.5, k.boomOut(a), kDelta);
        assertEquals(0.5, k.stickOut(a), kDelta);
        assertEquals(1.0, k.wristOut(a), kDelta);
        Translation3d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(2, t.getY(), kDelta);
        assertEquals(0, t.getZ(), kDelta);
    }

    @Test
    void testi3() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 1, 1, 1);
        Translation3d t = new Translation3d(0, 2, 0);
        LynxArmAngles a = k.inverse(t, 0, 0, 0);
        assertEquals(0, a.swingRad(), kDelta);
        assertEquals(Math.PI / 6, a.boomRad(), kDelta);
        assertEquals(2 * Math.PI / 3, a.stickRad(), kDelta);
        assertEquals(-Math.PI / 3, a.wristRad(), kDelta);
        assertEquals(0, a.twist, kDelta);
        assertEquals(0, a.grip, kDelta);
    }

    // wrist on the floor, ahead, swung counterclockwise 30 deg.
    @Test
    void testf4() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 1, 1, 1);
        LynxArmAngles a = new LynxArmAngles.Factory().fromRad(Math.PI / 6, Math.PI / 6, 2 * Math.PI / 3, -Math.PI / 3,
                0, 0);
        Translation3d t = k.forward(a);
        assertEquals(-1, t.getX(), kDelta);
        assertEquals(Math.sqrt(3), t.getY(), kDelta);
        assertEquals(0, t.getZ(), kDelta);
    }

    @Test
    void testi4() {
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory();
        LynxArmKinematics k = new LynxArmKinematics(factory, 1, 1, 1);
        Translation3d t = new Translation3d(-1, Math.sqrt(3), 0);
        LynxArmAngles a = k.inverse(t, 0, 0, 0);
        assertEquals(Math.PI / 6, a.swingRad(), kDelta);
        assertEquals(Math.PI / 6, a.boomRad(), kDelta);
        assertEquals(2 * Math.PI / 3, a.stickRad(), kDelta);
        assertEquals(-Math.PI / 3, a.wristRad(), kDelta);
        assertEquals(0, a.twist, kDelta);
        assertEquals(0, a.grip, kDelta);
    }

}