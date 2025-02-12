package org.team100.lib.follower;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.testing.Timeless;
import org.team100.lib.timing.TimingUtil;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryUtil100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class DriveMotionPlannerTest implements Timeless {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testFieldRelativeTrajectory() {
        List<Pose2d> waypoints = new ArrayList<>();
        List<Rotation2d> headings = new ArrayList<>();
        waypoints.add(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        headings.add(Rotation2d.fromDegrees(180));
        waypoints.add(new Pose2d(100, 4, Rotation2d.fromDegrees(0)));
        headings.add(Rotation2d.fromDegrees(180));
        waypoints.add(new Pose2d(196, 13, Rotation2d.fromDegrees(0)));
        headings.add(Rotation2d.fromDegrees(0));

        double start_vel = 0.0;
        double end_vel = 0.0;

        Path100 path = TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(waypoints, headings, 2, 0.25, 0.1);
        assertFalse(path.isEmpty());

        PathDistanceSampler view = new PathDistanceSampler(path);
        double stepSize = 2;
        TimingUtil u = new TimingUtil(Arrays.asList());
        Trajectory100 trajectory = u.timeParameterizeTrajectory(
                view,
                stepSize,
                start_vel,
                end_vel);

        TrajectoryFollower controller = new TrajectoryFollower(
                logger, new FollowerController(logger, 2.4, 2.4, 1.0, 1.0, 0.01, 0.02, 0.01, 0.02));

        controller.setTrajectory(trajectory);

        Pose2d pose = trajectory.sample(0).state().getPose();
        FieldRelativeVelocity velocity = FieldRelativeVelocity.zero();

        // double time = Takt.get()
        double mDt = 0.02;
        while (!controller.isDone()) {
            stepTime(0.02);
            velocity = controller.update(new SwerveModel(pose, velocity));
            pose = new Pose2d(
                    pose.getX() + velocity.x() * mDt,
                    pose.getY() + velocity.y() * mDt,
                    new Rotation2d(pose.getRotation().getRadians() + velocity.theta() * mDt));
        }

        // this should be exactly right but it's not.
        // TODO: it's because of the clock skew in the feedback, so fix that.
        assertEquals(195, pose.getTranslation().getX(), 1);
        assertEquals(13, pose.getTranslation().getY(), 0.4);
        assertEquals(0, pose.getRotation().getRadians(), 0.1);
    }
}