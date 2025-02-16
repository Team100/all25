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
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.TrajectoryToPose;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryMaker;

import edu.wpi.first.math.geometry.Pose2d;

class TrajectoryToPoseTest {
    private static final double kDelta = 0.001;
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    TrajectoryMaker maker = new TrajectoryMaker(constraints);

    @Test
    void testRestToRest() {
        TrajectoryToPose t = new TrajectoryToPose(false, maker);
        SwerveModel start = new SwerveModel(GeometryUtil.kPoseZero, new FieldRelativeVelocity(0, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 trajectory = t.apply(start, end);
        assertEquals(0.904, trajectory.duration(), kDelta);

        /** progress along trajectory */
        double m_timeS = 0;

        // initial velocity is zero.
        assertEquals(0, trajectory.sample(m_timeS).velocityM_S(), kDelta);

        double maxDriveVelocityM_S = swerveKinodynamics.getMaxDriveVelocityM_S();
        double maxDriveAccelerationM_S2 = swerveKinodynamics.getMaxDriveAccelerationM_S2();
        assertEquals(5, maxDriveVelocityM_S);
        assertEquals(10, maxDriveAccelerationM_S2);
        for (TimedPose p : trajectory.getPoints()) {
            assertTrue(p.velocityM_S() - 0.001 <= maxDriveVelocityM_S,
                    String.format("%f %f", p.velocityM_S(), maxDriveVelocityM_S));
            assertTrue(p.acceleration() - 0.001 <= maxDriveAccelerationM_S2,
                    String.format("%f %f", p.acceleration(), maxDriveAccelerationM_S2));
        }
    }

    @Test
    void testMovingToRest() {
        TrajectoryToPose t = new TrajectoryToPose(true, maker);
        SwerveModel start = new SwerveModel(GeometryUtil.kPoseZero, new FieldRelativeVelocity(1, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(0.744, traj.duration(), kDelta);
    }

    @Test
    void testBackingUp() {
        TrajectoryToPose t = new TrajectoryToPose(true, maker);
        SwerveModel start = new SwerveModel(GeometryUtil.kPoseZero, new FieldRelativeVelocity(-1, 0, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(0.877, traj.duration(), kDelta);
    }

    @Test
    void test2d() {
        TrajectoryToPose t = new TrajectoryToPose(true, maker);
        SwerveModel start = new SwerveModel(GeometryUtil.kPoseZero, new FieldRelativeVelocity(0, 1, 0));
        Pose2d end = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        Trajectory100 traj = t.apply(start, end);
        assertEquals(1.247, traj.duration(), kDelta);
    }
}
