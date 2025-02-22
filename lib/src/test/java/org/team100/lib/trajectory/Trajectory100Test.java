package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

class Trajectory100Test {
    private static final double kDelta = 0.001;
    private static final boolean DEBUG = false;

    @Test
    void testPreviewAndAdvance() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.get();
        Pose2d start = GeometryUtil.kPoseZero;
        Pose2d end = start.plus(new Transform2d(1, 0, GeometryUtil.kRotationZero));

        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        List<Pose2d> waypointsM = List.of(
                new Pose2d(currentTranslation, angleToGoal),
                new Pose2d(goalTranslation, angleToGoal));

        List<Rotation2d> headings = List.of(
                start.getRotation(),
                end.getRotation());

        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

        Trajectory100 trajectory = planner.restToRest(waypointsM, headings);

        TimedPose sample = trajectory.sample(0);
        assertEquals(0, sample.state().getPose().getX(), kDelta);

        sample = trajectory.sample(1);
        assertEquals(1, sample.state().getPose().getX(), kDelta);

        sample = trajectory.sample(2);
        assertEquals(1, sample.state().getPose().getX(), kDelta);

        sample = trajectory.sample(3);
        assertEquals(1, sample.state().getPose().getX(), kDelta);
    }

    @Test
    void testSample() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3();
        Pose2d start = GeometryUtil.kPoseZero;
        Pose2d end = start.plus(new Transform2d(1, 0, GeometryUtil.kRotationZero));

        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        List<Pose2d> waypointsM = List.of(
                new Pose2d(currentTranslation, angleToGoal),
                new Pose2d(goalTranslation, angleToGoal));

        List<Rotation2d> headings = List.of(
                start.getRotation(),
                end.getRotation());

        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

        Trajectory100 trajectory = planner.restToRest(waypointsM, headings);

        assertEquals(1.415, trajectory.duration(), kDelta);
        TimedPose sample = trajectory.sample(0);
        assertEquals(0, sample.state().getPose().getX(), kDelta);
        sample = trajectory.sample(1);
        assertEquals(0.828, sample.state().getPose().getX(), kDelta);
        sample = trajectory.sample(2);
        assertEquals(1, sample.state().getPose().getX(), kDelta);
    }

    /**
     * This is to hold the sample invariant while adding the index.
     */
    @Test
    void testSampleThoroughly() {

        List<Pose2d> waypointsM = List.of(
                new Pose2d(new Translation2d(), GeometryUtil.kRotationZero),
                new Pose2d(new Translation2d(1, 0), GeometryUtil.kRotationZero));

        List<Rotation2d> headings = List.of(
                GeometryUtil.kRotationZero,
                GeometryUtil.kRotationZero);

        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3();
        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

        Trajectory100 trajectory = planner.restToRest(waypointsM, headings);

        assertEquals(1.415, trajectory.duration(), kDelta);
        assertEquals(0.000, trajectory.sample(0).state().getPose().getX(), kDelta);
        assertEquals(0.010, trajectory.sample(0.1).state().getPose().getX(), kDelta);
        assertEquals(0.040, trajectory.sample(0.2).state().getPose().getX(), kDelta);
        assertEquals(0.090, trajectory.sample(0.3).state().getPose().getX(), kDelta);
        assertEquals(0.160, trajectory.sample(0.4).state().getPose().getX(), kDelta);
        assertEquals(0.250, trajectory.sample(0.5).state().getPose().getX(), kDelta);
        assertEquals(0.360, trajectory.sample(0.6).state().getPose().getX(), kDelta);
        assertEquals(0.490, trajectory.sample(0.7).state().getPose().getX(), kDelta);
        assertEquals(0.622, trajectory.sample(0.8).state().getPose().getX(), kDelta);
        assertEquals(0.735, trajectory.sample(0.9).state().getPose().getX(), kDelta);
        assertEquals(0.828, trajectory.sample(1).state().getPose().getX(), kDelta);
        assertEquals(0.901, trajectory.sample(1.1).state().getPose().getX(), kDelta);
        assertEquals(0.954, trajectory.sample(1.2).state().getPose().getX(), kDelta);
        assertEquals(0.987, trajectory.sample(1.3).state().getPose().getX(), kDelta);
        assertEquals(1.000, trajectory.sample(1.4).state().getPose().getX(), kDelta);
        assertEquals(1.000, trajectory.sample(1.5).state().getPose().getX(), kDelta);
    }

    @Test
    void testSampleThoroughlyWithRotation() {

        List<Pose2d> waypointsM = List.of(
                new Pose2d(new Translation2d(), GeometryUtil.kRotationZero),
                new Pose2d(new Translation2d(1, 0), GeometryUtil.kRotationZero));

        List<Rotation2d> headings = List.of(
                GeometryUtil.kRotationZero,
                GeometryUtil.kRotation90);

        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3();
        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

        Trajectory100 trajectory = planner.restToRest(waypointsM, headings);

        assertEquals(1.499, trajectory.duration(), kDelta);
        assertEquals(0.000, trajectory.sample(0).state().getPose().getX(), kDelta);
        assertEquals(0.010, trajectory.sample(0.1).state().getPose().getX(), kDelta);
        assertEquals(0.040, trajectory.sample(0.2).state().getPose().getX(), kDelta);
        assertEquals(0.090, trajectory.sample(0.3).state().getPose().getX(), kDelta);
        assertEquals(0.160, trajectory.sample(0.4).state().getPose().getX(), kDelta);
        assertEquals(0.250, trajectory.sample(0.5).state().getPose().getX(), kDelta);
        assertEquals(0.357, trajectory.sample(0.6).state().getPose().getX(), kDelta);
        assertEquals(0.454, trajectory.sample(0.7).state().getPose().getX(), kDelta);
        assertEquals(0.544, trajectory.sample(0.8).state().getPose().getX(), kDelta);
        assertEquals(0.643, trajectory.sample(0.9).state().getPose().getX(), kDelta);
        assertEquals(0.751, trajectory.sample(1).state().getPose().getX(), kDelta);
        assertEquals(0.841, trajectory.sample(1.1).state().getPose().getX(), kDelta);
        assertEquals(0.910, trajectory.sample(1.2).state().getPose().getX(), kDelta);
        assertEquals(0.960, trajectory.sample(1.3).state().getPose().getX(), kDelta);
        assertEquals(0.990, trajectory.sample(1.4).state().getPose().getX(), kDelta);
        assertEquals(1.000, trajectory.sample(1.5).state().getPose().getX(), kDelta);

    }

    /** Does the index help? No. */
    @Test
    void testSamplePerformance() {
        List<Pose2d> waypointsM = List.of(
                new Pose2d(new Translation2d(), GeometryUtil.kRotationZero),
                new Pose2d(new Translation2d(10, 0), GeometryUtil.kRotationZero),
                new Pose2d(new Translation2d(10, 10), GeometryUtil.kRotationZero));

        List<Rotation2d> headings = List.of(
                GeometryUtil.kRotationZero,
                GeometryUtil.kRotation90,
                GeometryUtil.kRotation180);

        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3();
        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

        Trajectory100 trajectory = planner.restToRest(waypointsM, headings);

        assertEquals(1851, trajectory.length());
        int reps = 500000;
        int times = 10;
        long start = System.nanoTime();
        for (int rep = 0; rep < reps; ++rep) {
            for (int t = 0; t < times; ++t) {
                trajectory.sample(0.1 * t);
            }
        }
        long end = System.nanoTime();
        long duration = end - start;
        if (DEBUG)
            Util.printf("duration (ns) total (ms) %.0f per sample (ns) %.2f\n",
                    0.000001 * duration, (double) duration / (reps * times));
    }

}
