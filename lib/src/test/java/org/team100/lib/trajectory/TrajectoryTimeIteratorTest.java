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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectoryTimeIteratorTest {
    private static final double kDelta = 0.001;

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

        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, constraints);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(trajectory);

        TimedPose sample = iter.preview(0);
        assertEquals(0, sample.state().getPose().getX(), kDelta);
        sample = iter.advance(0);
        assertEquals(0, sample.state().getPose().getX(), kDelta);

        sample = iter.preview(1);
        assertEquals(1, sample.state().getPose().getX(), kDelta);
        sample = iter.advance(1);
        assertEquals(1, sample.state().getPose().getX(), kDelta);

        sample = iter.preview(1);
        assertEquals(1, sample.state().getPose().getX(), kDelta);
        sample = iter.advance(1);
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

        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, constraints);

        TrajectoryTimeIterator sampler = new TrajectoryTimeIterator(trajectory);
        assertEquals(0, sampler.getStartS(), kDelta);
        assertEquals(1.415, sampler.getEndS(), kDelta);
        TimedPose sample = sampler.sample(0);
        assertEquals(0, sample.state().getPose().getX(), kDelta);
        sample = sampler.sample(1);
        assertEquals(0.828, sample.state().getPose().getX(), kDelta);
        sample = sampler.sample(2);
        assertEquals(1, sample.state().getPose().getX(), kDelta);

    }

}
