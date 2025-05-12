package org.team100.lib.motion.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.lynxmotion_arm.LynxArmKinematics.LynxArmConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class LynxArmKinematicsTest {
    private static final double kDelta = 0.001;

    @Test
    void testBadRotation() {
        // stretched out along x but with the wrong end rotation
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Pose3d t = new Pose3d(new Translation3d(2, 0, 1), new Rotation3d(0, 0, 1));
        assertThrows(IllegalArgumentException.class, () -> k.inverse(t));
    }

    @Test
    void testOutOfBounds() {
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Pose3d t = new Pose3d(new Translation3d(5, 0, 1), new Rotation3d(0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> k.inverse(t));
    }

    @Test
    void test1() {
        // stretched out along x
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Pose3d p = new Pose3d(new Translation3d(3, 0, 1), new Rotation3d());
        LynxArmConfig q = new LynxArmConfig(0, 0, 0, 0, 0);
        verify(k, p, q);
    }

    @Test
    void test2() {
        // stretched out along x
        LynxArmKinematics k = new LynxArmKinematics(0.5, 0.5, 0.5, 0.5);
        Pose3d p = new Pose3d(new Translation3d(1.5, 0, 0.5), new Rotation3d());
        LynxArmConfig q = new LynxArmConfig(0, 0, 0, 0, 0);
        verify(k, p, q);
    }

    @Test
    void test3() {
        // up at the shoulder, forward at the elbow
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Pose3d p = new Pose3d(new Translation3d(2, 0, 2), new Rotation3d());
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 2, Math.PI / 2, 0, 0);
        verify(k, p, q);
    }

    @Test
    void test4() {
        // boom and stick make an equilateral, wrist points ahead
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Pose3d p = new Pose3d(new Translation3d(2, 0, 1), new Rotation3d());
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 3, 2 * Math.PI / 3, -Math.PI / 3, 0);
        verify(k, p, q);
    }

    @Test
    void test5() {
        // boom and stick make an equilateral, wrist points ahead, with swing
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Pose3d p = new Pose3d(new Translation3d(Math.sqrt(3), 1, 1), new Rotation3d(0, 0, Math.PI / 6));
        LynxArmConfig q = new LynxArmConfig(Math.PI / 6, -Math.PI / 3, 2 * Math.PI / 3, -Math.PI / 3, 0);
        verify(k, p, q);
    }

    @Test
    void test6() {
        // wrist vertical
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 3, 2 * Math.PI / 3, Math.PI / 6, 0);
        Pose3d p = new Pose3d(new Translation3d(1, 0, 0), new Rotation3d(0, Math.PI / 2, 0));
        verify(k, p, q);
    }

    @Test
    void test7() {
        // wrist vertical with roll (which is yaw but inverted since it's pointing down)
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 3, 2 * Math.PI / 3, Math.PI / 6, Math.PI / 2);
        Pose3d p = new Pose3d(new Translation3d(1, 0, 0), new Rotation3d(0, Math.PI / 2, -Math.PI / 2));
        verify(k, p, q);
    }

    @Test
    void test8() {
        // wrist vertical with roll and swing
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        LynxArmConfig q = new LynxArmConfig(
                Math.PI / 3, -Math.PI / 3, 2 * Math.PI / 3, Math.PI / 6, Math.PI / 2);
        Pose3d p = new Pose3d(
                new Translation3d(0.5, Math.sqrt(3) / 2, 0),
                new Rotation3d(0, Math.PI / 2, -Math.PI / 6));
        verify(k, p, q);
    }

    @Test
    void test9() {
        // grip is on the swing axis, make the swing match end yaw
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Pose3d p = new Pose3d(new Translation3d(0, 0, 2), new Rotation3d(0, 0, -1));
        LynxArmConfig q = new LynxArmConfig(-1, -Math.PI, Math.PI / 2, Math.PI / 2, 0);
        verify(k, p, q);
    }

    @Test
    void test10() {
        // grip is on the swing axis, wrist pointing up => indeterminate
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Pose3d p = new Pose3d(new Translation3d(0, 0, 3), new Rotation3d(0, -Math.PI / 2, 0));
        LynxArmConfig q = new LynxArmConfig(
                OptionalDouble.empty(), -5 * Math.PI / 6, 2 * Math.PI / 3, -Math.PI / 3, OptionalDouble.empty());
        verify(k, p, q);
    }

    @Test
    void test11() {
        // arch
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Pose3d p = new Pose3d(new Translation3d(2, 0, 1), new Rotation3d(0, Math.PI / 3, 0));
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 3, Math.PI / 3, Math.PI / 3, 0);
        verify(k, p, q);
    }

    @Test
    void testPath() {
        // print the path from (1,1,0) to (1,-1,0). end effector on the table, wrist
        // vertical, no workspace twist
        LynxArmKinematics k = new LynxArmKinematics(1, 1, 1, 1);
        Translation3d start = new Translation3d(1, 1, 0);
        Translation3d end = new Translation3d(1, -1, 0);
        // end-effector rotation is fixed
        Rotation3d r = new Rotation3d(0, Math.PI / 2, 0);
        System.out.println("s, swing, boom, stick, wrist, twist");
        for (double s = 0; s <= 1.0; s += 0.05) {
            Translation3d t = start.interpolate(end, s);
            Pose3d p = new Pose3d(t, r);
            LynxArmConfig q = k.inverse(p);
            System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                    s, q.swing().getAsDouble(), q.boom(), q.stick(), q.wrist(), q.twist().getAsDouble());
        }
    }

    void verify(LynxArmKinematics k, Pose3d p, LynxArmConfig q) {
        verifyFwd(p, k.forward(q));
        verifyInv(q, k.inverse(p));
    }

    void verifyFwd(Pose3d expected, Pose3d actual) {
        assertEquals(expected.getX(), actual.getX(), kDelta, "fwd x");
        assertEquals(expected.getY(), actual.getY(), kDelta, "fwd y");
        assertEquals(expected.getZ(), actual.getZ(), kDelta, "fwd z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), kDelta, "fwd roll");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), kDelta, "fwd pitch");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), kDelta, "fwd yaw");
    }

    void verifyInv(LynxArmConfig expected, LynxArmConfig actual) {
        if (expected.swing().isPresent()) {
            assertEquals(expected.swing().getAsDouble(), actual.swing().getAsDouble(), kDelta, "inv swing");
        } else {
            assertTrue(actual.swing().isEmpty(), "inv swing");
        }
        assertEquals(expected.boom(), actual.boom(), kDelta, "inv boom");
        assertEquals(expected.stick(), actual.stick(), kDelta, "inv stick");
        assertEquals(expected.wrist(), actual.wrist(), kDelta, "inv wrist");
        if (expected.twist().isPresent()) {
            assertEquals(expected.twist().getAsDouble(), actual.twist().getAsDouble(), kDelta, "inv twist");
        } else {
            assertTrue(actual.twist().isEmpty());
        }
    }

}