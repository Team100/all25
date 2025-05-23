package org.team100.lib.motion.urdf;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Map;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class URDFUtilTest {
    @Test
    void test1() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        URDFModel.Joint j = URDFUtil.getJoint(m, "center_point");
        Transform3d t = URDFUtil.jointTransform(j, 1.0);
        // center point is just 5.5 cm down the x axis, fixed.
        assertEquals(0.055, t.getX(), 1e-3);
        assertEquals(0, t.getY(), 1e-3);
        assertEquals(0, t.getZ(), 1e-3);
        assertEquals(0, t.getRotation().getX(), 1e-3);
        assertEquals(0, t.getRotation().getY(), 1e-3);
        assertEquals(0, t.getRotation().getZ(), 1e-3);
    }

    @Test
    void test2() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                "shoulder_tilt", 0.0,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0,
                "center_point", 0.0);
        Map<String, Pose3d> poses = URDFUtil.forward(m, q);
        assertEquals(6, poses.size());
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "shoulder_tilt");
        verify(new Pose3d(0.14605, 0, 0.06731, new Rotation3d()), poses, "elbow_tilt");
        verify(new Pose3d(0.33337, 0, 0.06731, new Rotation3d()), poses, "wrist_tilt");
        verify(new Pose3d(0.36737, 0, 0.06731, new Rotation3d()), poses, "wrist_rotate");
        verify(new Pose3d(0.42237, 0, 0.06731, new Rotation3d()), poses, "center_point");
    }

    @Test
    void test3() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Pose3d end = new Pose3d(0.15, 0.0, 0.1, new Rotation3d(0, 0, 0));
        Map<String, Double> qMap = URDFUtil.inverse(m, "center_point", end);
        assertEquals(5, qMap.size());
        assertEquals(0, qMap.get("base_pan"), 1e-3);
        assertEquals(0, qMap.get("shoulder_tilt"), 1e-3);
        assertEquals(0, qMap.get("elbow_tilt"), 1e-3);
        assertEquals(0, qMap.get("wrist_tilt"), 1e-3);
        assertEquals(0, qMap.get("wrist_rotate"), 1e-3);
    }

    void verify(Pose3d expected, Map<String, Pose3d> poses, String name) {
        Pose3d actual = poses.get(name);
        assertEquals(expected.getX(), actual.getX(), 1e-3, name + " x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, name + " y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, name + " z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, name + " rot x");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, name + " rot y");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, name + " rot z");
    }
}
