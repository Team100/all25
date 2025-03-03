package org.team100.lib.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.spline.HolonomicSpline;
import org.team100.lib.spline.SplineGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class Path100Test {
    private static final double kDelta = 0.001;

    private static final List<Rotation2d> kHeadings = Arrays.asList(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(30),
            GeometryUtil.fromDegrees(60),
            GeometryUtil.fromDegrees(90));

    private static final List<Pose2dWithMotion> kWaypoints = Arrays.asList(
            new Pose2dWithMotion(new Pose2d(new Translation2d(0.0, 0.0), kHeadings.get(0))),
            new Pose2dWithMotion(new Pose2d(new Translation2d(24.0, 0.0), kHeadings.get(1))),
            new Pose2dWithMotion(new Pose2d(new Translation2d(36.0, 12.0), kHeadings.get(2))),
            new Pose2dWithMotion(new Pose2d(new Translation2d(60.0, 12.0), kHeadings.get(3))));

    @Test
    void testEmpty() {
        List<HolonomicSpline> splines = new ArrayList<>();
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = new Path100(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
        assertEquals(0, path.length(), 0.001);
    }

    @Test
    void testSimple() {
        // spline is in the x direction, no curvature.
        HolonomicSpline spline = new HolonomicSpline(
                new Pose2d(),
                new Pose2d(1, 0, new Rotation2d()),
                new Rotation2d(), new Rotation2d()) {

            @Override
            public Translation2d getPoint(double t) {
                return new Translation2d(t, 0);
            }

            @Override
            public Rotation2d getHeading(double t) {
                return Rotation2d.kZero;
            }

            @Override
            public Optional<Rotation2d> getCourse(double t) {
                return Optional.of(Rotation2d.kZero);
            }

            @Override
            public double getDHeading(double t) {
                return 0;
            }

            @Override
            public double getCurvature(double t) {
                return 0;
            }

            @Override
            public double getDCurvature(double t) {
                return 0;
            }

            @Override
            public double getVelocity(double t) {
                return 1;
            }
        };
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(spline);
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = new Path100(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
        assertEquals(2, path.length(), 0.001);
        {
            Pose2dWithMotion pose = path.getInterpolated(0);
            Pose2d pose2d = pose.getPose();
            assertEquals(0, pose2d.getX(), 0.001);
            assertEquals(0, pose2d.getY(), 0.001);
            assertEquals(0, pose2d.getRotation().getRadians(), 0.001);
        }
        {
            Pose2dWithMotion pose = path.getInterpolated(0.5);
            Pose2d pose2d = pose.getPose();
            assertEquals(0.5, pose2d.getX(), 0.001);
            assertEquals(0, pose2d.getY(), 0.001);
            assertEquals(0, pose2d.getRotation().getRadians(), 0.001);
        }
        {
            Pose2dWithMotion pose = path.getInterpolated(1);
            Pose2d pose2d = pose.getPose();
            assertEquals(1.0, pose2d.getX(), 0.001);
            assertEquals(0, pose2d.getY(), 0.001);
            assertEquals(0, pose2d.getRotation().getRadians(), 0.001);
        }
    }

    @Test
    void testConstruction() {
        Path100 traj = new Path100(kWaypoints);
        assertFalse(traj.isEmpty());
        assertEquals(4, traj.length());
    }

    @Test
    void testStateAccessors() {
        Path100 traj = new Path100(kWaypoints);

        assertEquals(kWaypoints.get(0), traj.getPoint(0));
        assertEquals(kWaypoints.get(1), traj.getPoint(1));
        assertEquals(kWaypoints.get(2), traj.getPoint(2));
        assertEquals(kWaypoints.get(3), traj.getPoint(3));

        assertEquals(kHeadings.get(0), traj.getPoint(0).getHeading());
        assertEquals(kHeadings.get(1), traj.getPoint(1).getHeading());
        assertEquals(kHeadings.get(2), traj.getPoint(2).getHeading());
        assertEquals(kHeadings.get(3), traj.getPoint(3).getHeading());
    }

    @Test
    void testInterpolateExact() {
        Path100 traj = new Path100(kWaypoints);

        Pose2dWithMotion interpolated0 = traj.getInterpolated(0.0);
        assertEquals(0, interpolated0.getPose().getX(), kDelta);
        assertEquals(0, interpolated0.getPose().getY(), kDelta);
        assertEquals(0, interpolated0.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion interpolated1 = traj.getInterpolated(1.0);
        assertEquals(24, interpolated1.getPose().getX(), kDelta);
        assertEquals(0, interpolated1.getPose().getY(), kDelta);
        assertEquals(30, interpolated1.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion interpolated2 = traj.getInterpolated(2.0);
        assertEquals(36, interpolated2.getPose().getX(), kDelta);
        assertEquals(12, interpolated2.getPose().getY(), kDelta);
        assertEquals(60, interpolated2.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion interpolated3 = traj.getInterpolated(3.0);
        assertEquals(60, interpolated3.getPose().getX(), kDelta);
        assertEquals(12, interpolated3.getPose().getY(), kDelta);
        assertEquals(90, interpolated3.getHeading().getDegrees(), kDelta);
    }

    /**
     * The constant-twist arcs between states mean these samples don't quite match
     * the straight-line paths.
     */
    @Test
    void testInterpolateBetween() {
        Path100 traj = new Path100(kWaypoints);

        Pose2dWithMotion interpolated025 = traj.getInterpolated(0.25);
        assertEquals(5.948, interpolated025.getTranslation().getX(), kDelta);
        assertEquals(-1.183, interpolated025.getTranslation().getY(), kDelta);
        assertEquals(7.5, interpolated025.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion interpolated150 = traj.getInterpolated(1.5);
        assertEquals(30.789, interpolated150.getTranslation().getX(), kDelta);
        assertEquals(5.210, interpolated150.getTranslation().getY(), kDelta);
        assertEquals(45, interpolated150.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion interpolated275 = traj.getInterpolated(2.75);
        assertEquals(54.051, interpolated275.getTranslation().getX(), kDelta);
        assertEquals(10.816, interpolated275.getTranslation().getY(), kDelta);
        assertEquals(82.5, interpolated275.getHeading().getDegrees(), kDelta);
    }
}
