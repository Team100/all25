package org.team100.lib.motion.arm23;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

class ArmKinematicsTest {
    private static final double kDelta = 0.001;

    @Test
    void testForward1() {
        ArmKinematics23 k = new ArmKinematics23(0.93, 0.92);// like 2023
        ArmAngles23 a = new ArmAngles23(0, Math.PI / 2);
        Translation2d t = k.forward(a);
        assertEquals(0.93, t.getX(), kDelta); // lower arm length
        assertEquals(0.92, t.getY(), kDelta); // upper arm length
    }

    @Test
    void testInverse1() {
        ArmKinematics23 k = new ArmKinematics23(0.93, 0.92);// like 2023
        Translation2d t = new Translation2d(0.93, 0.92);
        ArmAngles23 a = k.inverse(t);
        assertEquals(0, a.th1(), kDelta);
        assertEquals(Math.PI / 2, a.th2(), kDelta);
    }

    @Test
    void testForward2() {
        ArmKinematics23 k = new ArmKinematics23(0.93, 0.92);// like 2023
        ArmAngles23 a = new ArmAngles23(Math.PI / 4, Math.PI / 2);
        Translation2d t = k.forward(a);
        assertEquals(0.658, t.getX(), kDelta); // 0.93 * sqrt(2)/2
        assertEquals(1.578, t.getY(), kDelta); // 0.92 + 0.93 * sqrt(2)/2
    }

    @Test
    void testInverse2() {
        ArmKinematics23 k = new ArmKinematics23(0.93, 0.92);// like 2023
        Translation2d t = new Translation2d(0.658, 1.578);
        ArmAngles23 a = k.inverse(t);
        assertEquals(Math.PI / 4, a.th1(), kDelta);
        assertEquals(Math.PI / 2, a.th2(), kDelta);
    }

    @Test
    void testForward3() {
        ArmKinematics23 k = new ArmKinematics23(0.93, 0.92); // like 2023
        ArmAngles23 a = new ArmAngles23(Math.PI / 4, Math.PI / 4);
        Translation2d t = k.forward(a);
        assertEquals(1.308, t.getX(), kDelta); // (0.92 + 0.93) * sqrt(2)/2
        assertEquals(1.308, t.getY(), kDelta); // (0.92 + 0.93) * sqrt(2)/2
    }

    @Test
    void testInverse3() {
        ArmKinematics23 k = new ArmKinematics23(0.93, 0.92);// like 2023
        Translation2d t = new Translation2d(1.308147, 1.308147);
        ArmAngles23 a = k.inverse(t);
        // the angles here are very sensitive because the elbow is fully extended.
        assertEquals(Math.PI / 4, a.th1(), kDelta);
        assertEquals(Math.PI / 4, a.th2(), kDelta);
    }

    @Test
    void testForward4() {
        ArmKinematics23 k = new ArmKinematics23(1, 1); // unit length
        ArmAngles23 a = new ArmAngles23(0, Math.PI);
        Translation2d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(0, t.getY(), kDelta);
    }

    @Test
    void testInverse4() {
        ArmKinematics23 k = new ArmKinematics23(1, 1); // unit length
        Translation2d t = new Translation2d(0, 0);
        ArmAngles23 a = k.inverse(t);
        assertNull(a); // this case is underdetermined
    }

    @Test
    void testForward5() {
        ArmKinematics23 k = new ArmKinematics23(1, 1); // unit length
        ArmAngles23 a = new ArmAngles23(Math.PI / 4, 3 * Math.PI / 4);
        Translation2d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(1.414, t.getY(), kDelta); // sqrt(2)
    }

    @Test
    void testInverse5() {
        ArmKinematics23 k = new ArmKinematics23(1, 1);// unit length
        Translation2d t = new Translation2d(0, 1.414);// sqrt(2)
        ArmAngles23 a = k.inverse(t);
        assertEquals(Math.PI / 4, a.th1(), kDelta);
        assertEquals(3 * Math.PI / 4, a.th2(), kDelta);
    }

    @Test
    void testInverseUnreachable() {
        ArmKinematics23 k = new ArmKinematics23(1, 1); // unit length
        Translation2d t = new Translation2d(2, 2);
        ArmAngles23 a = k.inverse(t);
        assertNull(a); // this case is unreachable
    }

    // these are for the inverse velocity.

    @Test
    void testInverseVelocity() {
        // unit length
        final ArmKinematics23 k = new ArmKinematics23(1, 1);
        // both joints straight up
        final ArmAngles23 thetas = new ArmAngles23(0, 0);
        // not moving
        Translation2d dXY = new Translation2d(0, 0);
        ArmAngles23 dthetas = k.inverseVel(thetas, dXY);
        assertEquals(0, dthetas.th1(), kDelta);
        assertEquals(0, dthetas.th2(), kDelta);

        // this this yields the wrong number because the arm is straight
        dXY = new Translation2d(1, 0);
        dthetas = k.inverseVel(thetas, dXY);
        assertEquals(0, dthetas.th1(), kDelta);
        assertEquals(0, dthetas.th2(), kDelta);
    }

    @Test
    void testInverseVelocity2() {
        // unit length
        final ArmKinematics23 k = new ArmKinematics23(1, 1);
        // proximal +x, distal +y
        final ArmAngles23 thetas = new ArmAngles23(0, Math.PI / 2);
        // not moving
        Translation2d dXY = new Translation2d(0, 0);
        ArmAngles23 dthetas = k.inverseVel(thetas, dXY);
        assertEquals(0, dthetas.th1(), kDelta);
        assertEquals(0, dthetas.th2(), kDelta);

        // x motion requires distal movement only
        dXY = new Translation2d(1, 0);
        dthetas = k.inverseVel(thetas, dXY);
        assertEquals(0, dthetas.th1(), kDelta);
        assertEquals(-1, dthetas.th2(), kDelta);

        // y motion requires proximal movement only
        dXY = new Translation2d(0, 1);
        dthetas = k.inverseVel(thetas, dXY);
        assertEquals(1, dthetas.th1(), kDelta);
        assertEquals(0, dthetas.th2(), kDelta);

        // combined motion
        dXY = new Translation2d(1, 1);
        dthetas = k.inverseVel(thetas, dXY);
        assertEquals(1, dthetas.th1(), kDelta);
        assertEquals(-1, dthetas.th2(), kDelta);

        // faster
        dXY = new Translation2d(2, 2);
        dthetas = k.inverseVel(thetas, dXY);
        assertEquals(2, dthetas.th1(), kDelta);
        assertEquals(-2, dthetas.th2(), kDelta);
    }

    @Test
    void testInverseVelocity3() {
        // unit length
        final ArmKinematics23 k = new ArmKinematics23(1, 1);
        // proximal +x, distal at 45 degrees
        final ArmAngles23 thetas = new ArmAngles23(0, Math.PI / 4);
        // not moving
        Translation2d dXY = new Translation2d(0, 0);
        ArmAngles23 dthetas = k.inverseVel(thetas, dXY);
        assertEquals(0, dthetas.th1(), kDelta);
        assertEquals(0, dthetas.th2(), kDelta);

        // x motion
        dXY = new Translation2d(1, 0);
        dthetas = k.inverseVel(thetas, dXY);
        assertEquals(1, dthetas.th1(), kDelta);
        assertEquals(-1.414, dthetas.th2(), kDelta);

        // y motion is the same
        dXY = new Translation2d(0, 1);
        dthetas = k.inverseVel(thetas, dXY);
        assertEquals(1, dthetas.th1(), kDelta);
        assertEquals(0, dthetas.th2(), kDelta);

        // directly away from the origin should be equal joint motion
        dXY = new Translation2d(1, 0.414);
        dthetas = k.inverseVel(thetas, dXY);
        assertEquals(1.414, dthetas.th1(), kDelta);
        assertEquals(-1.414, dthetas.th2(), kDelta);
    }

    @Test
    void testdtheta1() {
        ArmKinematics23 kinematics = new ArmKinematics23(.93, .92);
        ArmAngles23 thetas = new ArmAngles23(0, Math.PI / 2);
        Translation2d dXY = new Translation2d(.92, 0);
        ArmAngles23 dtheta = kinematics.inverseVel(thetas, dXY);
        assertEquals(0, dtheta.th1(), kDelta);
        assertEquals(-1, dtheta.th2(), kDelta);
    }

    @Test
    void testdtheta2() {
        ArmKinematics23 kinematics = new ArmKinematics23(1, 1);
        ArmAngles23 thetas = new ArmAngles23(Math.PI / 4, 0);
        Translation2d dXY = new Translation2d(1, 0);
        ArmAngles23 dtheta = kinematics.inverseVel(thetas, dXY);
        assertEquals(1, dtheta.th2(), kDelta, "UPPER THETA VALUE: " + dtheta.th2());
        assertEquals(-1.414, dtheta.th1(), kDelta, "LOWER THETA VALUE: " + dtheta.th1());
    }

}
