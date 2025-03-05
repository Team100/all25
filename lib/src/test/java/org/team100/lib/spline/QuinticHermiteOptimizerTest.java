package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.HolonomicPose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class QuinticHermiteOptimizerTest {
    private static double kEpsilon = 1e-12;

    @Test
    void test() {
        HolonomicPose2d a = new HolonomicPose2d(
                new Translation2d(0, 100), new Rotation2d(), Rotation2d.fromDegrees(270));
        HolonomicPose2d b = new HolonomicPose2d(
                new Translation2d(50, 0), new Rotation2d(), Rotation2d.fromDegrees(0));
        HolonomicPose2d c = new HolonomicPose2d(
                new Translation2d(100, 100), new Rotation2d(), Rotation2d.fromDegrees(90));

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(a, b));
        splines.add(new HolonomicSpline(b, c));

        assertTrue(SplineUtil.optimizeSpline(splines) < 0.014);

        HolonomicPose2d d = new HolonomicPose2d(
                new Translation2d(0, 0), new Rotation2d(), Rotation2d.fromDegrees(90));
        HolonomicPose2d e = new HolonomicPose2d(
                new Translation2d(0, 50), new Rotation2d(), Rotation2d.fromDegrees(0));
        HolonomicPose2d f = new HolonomicPose2d(
                new Translation2d(100, 50), new Rotation2d(), Rotation2d.fromDegrees(-90));
        HolonomicPose2d g = new HolonomicPose2d(
                new Translation2d(100, 0), new Rotation2d(), Rotation2d.fromDegrees(-180));

        List<HolonomicSpline> splines1 = new ArrayList<>();
        splines1.add(new HolonomicSpline(d, e));
        splines1.add(new HolonomicSpline(e, f));
        splines1.add(new HolonomicSpline(f, g));

        assertEquals(0.54, SplineUtil.optimizeSpline(splines1), 0.01);

        HolonomicPose2d h = new HolonomicPose2d(
                new Translation2d(0, 0), new Rotation2d(), Rotation2d.fromDegrees(0));
        HolonomicPose2d i = new HolonomicPose2d(
                new Translation2d(50, 0), new Rotation2d(), Rotation2d.fromDegrees(0));
        HolonomicPose2d j = new HolonomicPose2d(
                new Translation2d(100, 50), new Rotation2d(), Rotation2d.fromDegrees(45));
        HolonomicPose2d k = new HolonomicPose2d(
                new Translation2d(150, 0), new Rotation2d(), Rotation2d.fromDegrees(270));
        HolonomicPose2d l = new HolonomicPose2d(
                new Translation2d(150, -50), new Rotation2d(), Rotation2d.fromDegrees(270));

        List<HolonomicSpline> splines2 = new ArrayList<>();
        splines2.add(new HolonomicSpline(h, i));
        splines2.add(new HolonomicSpline(i, j));
        splines2.add(new HolonomicSpline(j, k));
        splines2.add(new HolonomicSpline(k, l));

        assertTrue(SplineUtil.optimizeSpline(splines2) < 0.05);
        assertEquals(0.0, splines2.get(0).getCurvature(1.0), kEpsilon);
        assertEquals(0.0, splines2.get(2).getCurvature(1.0), kEpsilon);
    }

    @Test
    void testHolonomic() {
        HolonomicPose2d a = new HolonomicPose2d(
                new Translation2d(0, 100), new Rotation2d(), Rotation2d.fromDegrees(270));
        HolonomicPose2d b = new HolonomicPose2d(
                new Translation2d(50, 0), new Rotation2d(Math.PI / 2), Rotation2d.fromDegrees(0));
        HolonomicPose2d c = new HolonomicPose2d(
                new Translation2d(100, 100), new Rotation2d(Math.PI), Rotation2d.fromDegrees(90));

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(a, b));
        splines.add(new HolonomicSpline(b, c));

        assertTrue(SplineUtil.optimizeSpline(splines) < 0.014);

        HolonomicPose2d d = new HolonomicPose2d(
                new Translation2d(0, 0), new Rotation2d(), Rotation2d.fromDegrees(90));
        HolonomicPose2d e = new HolonomicPose2d(
                new Translation2d(0, 50), new Rotation2d(Math.PI / 2), Rotation2d.fromDegrees(0));
        HolonomicPose2d f = new HolonomicPose2d(
                new Translation2d(100, 50), new Rotation2d(Math.PI), Rotation2d.fromDegrees(-90));
        HolonomicPose2d g = new HolonomicPose2d(
                new Translation2d(100, 0), new Rotation2d(), Rotation2d.fromDegrees(-180));

        List<HolonomicSpline> splines1 = new ArrayList<>();
        splines1.add(new HolonomicSpline(d, e));
        splines1.add(new HolonomicSpline(e, f));
        splines1.add(new HolonomicSpline(f, g));

        assertEquals(0.54, SplineUtil.optimizeSpline(splines1), 0.01);

        HolonomicPose2d h = new HolonomicPose2d(
                new Translation2d(0, 0), new Rotation2d(), Rotation2d.fromDegrees(0));
        HolonomicPose2d i = new HolonomicPose2d(
                new Translation2d(50, 0), new Rotation2d(Math.PI / 2), Rotation2d.fromDegrees(0));
        HolonomicPose2d j = new HolonomicPose2d(
                new Translation2d(100, 50), new Rotation2d(Math.PI), Rotation2d.fromDegrees(45));
        HolonomicPose2d k = new HolonomicPose2d(
                new Translation2d(150, 0), new Rotation2d(), Rotation2d.fromDegrees(270));
        HolonomicPose2d l = new HolonomicPose2d(
                new Translation2d(150, -50), new Rotation2d(Math.PI / 2), Rotation2d.fromDegrees(270));

        List<HolonomicSpline> splines2 = new ArrayList<>();
        splines2.add(new HolonomicSpline(h, i));
        splines2.add(new HolonomicSpline(i, j));
        splines2.add(new HolonomicSpline(j, k));
        splines2.add(new HolonomicSpline(k, l));

        assertTrue(SplineUtil.optimizeSpline(splines2) < 0.05);
        assertEquals(0.0, splines2.get(0).getCurvature(1.0), kEpsilon);
        assertEquals(0.0, splines2.get(2).getCurvature(1.0), kEpsilon);

    }

}
