package org.team100.lib.motion.urdf;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashMap;
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
                "wrist_rotate", 0.0);
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
    void test2a() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                // positive = pan left, so extent is +x +y
                "base_pan", Math.PI / 4,
                "shoulder_tilt", 0.0,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0);
        Map<String, Pose3d> poses = URDFUtil.forward(m, q);
        assertEquals(6, poses.size());
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "base_pan");
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "shoulder_tilt");
        verify(new Pose3d(0.10327, 0.10327, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "elbow_tilt");
        verify(new Pose3d(0.23573, 0.23573, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "wrist_tilt");
        verify(new Pose3d(0.25977, 0.25977, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "wrist_rotate");
        verify(new Pose3d(0.29866, 0.29866, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "center_point");
    }

    @Test
    void test2b() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                // negative = tilt up
                "shoulder_tilt", -Math.PI / 4,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0);
        Map<String, Pose3d> poses = URDFUtil.forward(m, q);
        assertEquals(6, poses.size());
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, -Math.PI / 4, 0)), poses, "shoulder_tilt");
        verify(new Pose3d(0.10372, 0, 0.17058, new Rotation3d(0, -Math.PI / 4, 0)), poses, "elbow_tilt");
        verify(new Pose3d(0.23573, 0, 0.30304, new Rotation3d(0, -Math.PI / 4, 0)), poses, "wrist_tilt");
        verify(new Pose3d(0.25977, 0, 0.32708, new Rotation3d(0, -Math.PI / 4, 0)), poses, "wrist_rotate");
        verify(new Pose3d(0.29866, 0, 0.36597, new Rotation3d(0, -Math.PI / 4, 0)), poses, "center_point");
    }

    @Test
    void test2c() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                // elbow tilts only down
                "shoulder_tilt", -Math.PI / 4,
                // so the rest should be horizontal
                "elbow_tilt", Math.PI / 4,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0);
        Map<String, Pose3d> poses = URDFUtil.forward(m, q);
        assertEquals(6, poses.size());
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, -Math.PI / 4, 0)), poses, "shoulder_tilt");
        verify(new Pose3d(0.10372, 0, 0.17058, new Rotation3d()), poses, "elbow_tilt");
        verify(new Pose3d(0.29060, 0, 0.17058, new Rotation3d()), poses, "wrist_tilt");
        verify(new Pose3d(0.32460, 0, 0.17058, new Rotation3d()), poses, "wrist_rotate");
        verify(new Pose3d(0.37960, 0, 0.17058, new Rotation3d()), poses, "center_point");
    }

    @Test
    void test2d() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                "shoulder_tilt", 0.0,
                "elbow_tilt", 0.0,
                // wrist tilt up a bit
                "wrist_tilt", -Math.PI / 4,
                "wrist_rotate", 0.0);
        Map<String, Pose3d> poses = URDFUtil.forward(m, q);
        assertEquals(6, poses.size());
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, 0, 0)), poses, "shoulder_tilt");
        verify(new Pose3d(0.14605, 0, 0.06731, new Rotation3d()), poses, "elbow_tilt");
        verify(new Pose3d(0.33337, 0, 0.06731, new Rotation3d(0, -Math.PI / 4, 0)), poses, "wrist_tilt");
        verify(new Pose3d(0.35741, 0, 0.09135, new Rotation3d(0, -Math.PI / 4, 0)), poses, "wrist_rotate");
        verify(new Pose3d(0.39631, 0, 0.13024, new Rotation3d(0, -Math.PI / 4, 0)), poses, "center_point");
    }

    @Test
    void test2e() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                "shoulder_tilt", 0.0,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                // rotate clockwise looking down x
                "wrist_rotate", Math.PI / 4);
        Map<String, Pose3d> poses = URDFUtil.forward(m, q);
        assertEquals(6, poses.size());
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "shoulder_tilt");
        verify(new Pose3d(0.14605, 0, 0.06731, new Rotation3d()), poses, "elbow_tilt");
        verify(new Pose3d(0.33337, 0, 0.06731, new Rotation3d()), poses, "wrist_tilt");
        verify(new Pose3d(0.36737, 0, 0.06731, new Rotation3d(Math.PI / 4, 0, 0)), poses, "wrist_rotate");
        verify(new Pose3d(0.42237, 0, 0.06731, new Rotation3d(Math.PI / 4, 0, 0)), poses, "center_point");
    }

    @Test
    void testBase() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                // positive = pan left, so extent is +x +y
                "base_pan", Math.PI / 4,
                "shoulder_tilt", 0.0,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0,
                "center_point", 0.0);
        Map<String, Pose3d> poses = new HashMap<>();
        Pose3d pose = URDFUtil.forward(m, poses, "base_pan", q);
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), pose, "base_pan");
    }

    @Test
    void testShoulder() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                // negative = tilt up
                "shoulder_tilt", -Math.PI / 4,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0,
                "center_point", 0.0);
        Map<String, Pose3d> poses = new HashMap<>();
        Pose3d pose = URDFUtil.forward(m, poses, "shoulder_tilt", q);
        verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, -Math.PI / 4, 0)), pose, "shoulder_tilt");
    }

    @Test
    void testElbow() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                // elbow tilts only down, so first tilt up a bit
                "shoulder_tilt", -Math.PI / 4,
                // so the rest should be horizontal
                "elbow_tilt", Math.PI / 4,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0,
                "center_point", 0.0);
        Map<String, Pose3d> poses = new HashMap<>();
        Pose3d pose = URDFUtil.forward(m, poses, "elbow_tilt", q);
        verify(new Pose3d(0.10372, 0, 0.17058, new Rotation3d(0, 0, 0)), pose, "shoulder_tilt");
    }

    @Test
    void testElbowTransform() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        URDFModel.Joint j = URDFUtil.getJoint(m, "elbow_tilt");
        Transform3d t = URDFUtil.jointTransform(j, Math.PI / 4);
        verify(new Transform3d(0.14605, 0, 0, new Rotation3d(0, Math.PI / 4, 0)), t);
    }

    @Test
    void testCenterPoint() {
        // this is what the inverse kinematics below should come up with
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                "shoulder_tilt", -2.522,
                "elbow_tilt", 2.804,
                "wrist_tilt", -0.282,
                "wrist_rotate", 0.0,
                "center_point", 0.0);
        Map<String, Pose3d> poses = new HashMap<>();
        Pose3d pose = URDFUtil.forward(m, poses, "center_point", q);
        print(pose);
        verify(new Pose3d(0.15, 0, 0.1, new Rotation3d(0, 0, 0)), pose, "center_point");
    }

    @Test
    void test3() {
        // this problem requires the dx limit, otherwise it oscillates
        // far from the solution.
        URDFModel.Robot m = URDFAL5D.ROBOT;
        Pose3d end = new Pose3d(0.15, 0.0, 0.1, new Rotation3d(0, 0, 0));
        Map<String, Double> qMap = URDFUtil.inverse(m, "center_point", end);
        assertEquals(5, qMap.size());
        assertEquals(0, qMap.get("base_pan"), 1e-3);
        assertEquals(-2.522, qMap.get("shoulder_tilt"), 1e-3);
        assertEquals(2.804, qMap.get("elbow_tilt"), 1e-3);
        assertEquals(-0.282, qMap.get("wrist_tilt"), 1e-3);
        assertEquals(0, qMap.get("wrist_rotate"), 1e-3);
    }

    void verify(Pose3d expected, Map<String, Pose3d> poses, String name) {
        Pose3d actual = poses.get(name);
        verify(expected, actual, name);
    }

    void verify(Pose3d expected, Pose3d actual, String name) {
        assertEquals(expected.getX(), actual.getX(), 1e-3, name + " x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, name + " y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, name + " z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, name + " rot x");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, name + " rot y");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, name + " rot z");
    }

    void verify(Transform3d expected, Transform3d actual) {
        assertEquals(expected.getX(), actual.getX(), 1e-3, " x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, " y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, " z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, " rot x");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, " rot y");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, " rot z");
    }

    void print(Pose3d p) {
        System.out.printf("x %6.3f y %6.3f z %6.3f r %6.3f p %6.3f y %6.3f\n",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }
}
