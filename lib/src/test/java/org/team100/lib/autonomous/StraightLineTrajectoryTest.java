package org.team100.lib.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.trajectory.TrajectoryPoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;

import edu.wpi.first.math.geometry.Pose2d;

class StraightLineTrajectoryTest {
    private static final double kDelta = 0.001;
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    TrajectoryMaker maker = new TrajectoryMaker(constraints);

    @Test
    void testRestToRest() {
        StraightLineTrajectory t = new StraightLineTrajectory(false, maker);
        SwerveModel start = new SwerveModel(GeometryUtil.kPoseZero, new FieldRelativeVelocity(0, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(0.904, traj.getTotalTimeSeconds(), kDelta);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(traj));
        // initial velocity is zero.
        assertEquals(0, iter.getSample().get().state().velocityM_S(), kDelta);

        double maxDriveVelocityM_S = swerveKinodynamics.getMaxDriveVelocityM_S();
        double maxDriveAccelerationM_S2 = swerveKinodynamics.getMaxDriveAccelerationM_S2();
        assertEquals(5, maxDriveVelocityM_S);
        assertEquals(10, maxDriveAccelerationM_S2);
        for (TrajectoryPoint p : traj.getPoints()) {
            assertTrue(p.state().velocityM_S() - 0.001 <= maxDriveVelocityM_S,
                    String.format("%f %f", p.state().velocityM_S(), maxDriveVelocityM_S));
            assertTrue(p.state().acceleration() - 0.001 <= maxDriveAccelerationM_S2,
                    String.format("%f %f", p.state().acceleration(), maxDriveAccelerationM_S2));
        }
    }

    @Test
    void testMovingToRest() {
        StraightLineTrajectory t = new StraightLineTrajectory(true, maker);
        SwerveModel start = new SwerveModel(GeometryUtil.kPoseZero, new FieldRelativeVelocity(1, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(0.744, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void testBackingUp() {
        StraightLineTrajectory t = new StraightLineTrajectory(true, maker);
        SwerveModel start = new SwerveModel(GeometryUtil.kPoseZero, new FieldRelativeVelocity(-1, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(0.877, traj.getTotalTimeSeconds(), kDelta);
    }

    @Test
    void test2d() {
        StraightLineTrajectory t = new StraightLineTrajectory(true, maker);
        SwerveModel start = new SwerveModel(GeometryUtil.kPoseZero, new FieldRelativeVelocity(0, 1, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(1.247, traj.getTotalTimeSeconds(), kDelta);
    }
}
