package org.team100.lib.follower;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.timing.TimingUtil;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryGenerator100;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.trajectory.TrajectoryUtil100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class DriveMotionPlannerTest {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest3();

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

        Path100 traj = new Path100();
        assertTrue(traj.isEmpty());
        assertEquals(0, traj.length());

        // Set states at construction time.
        traj = TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(waypoints, headings, 2, 0.25, 0.1);
        assertFalse(traj.isEmpty());

        var view = new PathDistanceSampler(traj);
        var stepSize = 2;
        TimingUtil u = new TimingUtil(Arrays.asList());
        Trajectory100 timed_trajectory = u.timeParameterizeTrajectory(
                view,
                stepSize,
                start_vel,
                end_vel);

        TrajectoryFollower controller = new TrajectoryFollower(
                logger, 2.4, 2.4, 1.0, 1.0);

        TrajectoryTimeIterator traj_iterator = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(timed_trajectory));
        controller.setTrajectory(traj_iterator);

        Pose2d pose = timed_trajectory.getPoint(0).state().state().getPose();
        FieldRelativeVelocity velocity = FieldRelativeVelocity.zero();

        double time = 0.0;
        double mDt = 0.005;
        while (!controller.isDone()) {
            velocity = controller.update(time, new SwerveModel(pose, velocity));
            pose = new Pose2d(
                    pose.getX() + velocity.x() * mDt,
                    pose.getY() + velocity.y() * mDt,
                    new Rotation2d(pose.getRotation().getRadians() + velocity.theta() * mDt));
            time += mDt;
        }
        // this should be exactly right but it's not.
        // TODO: it's because of the clock skew in the feedback, so fix that.
        assertEquals(196, pose.getTranslation().getX(), 0.6);
        assertEquals(13, pose.getTranslation().getY(), 0.4);
        assertEquals(0, pose.getRotation().getRadians(), 0.05);
    }

    @Test
    void testAllFieldRelativeTrajectories() {
        // note no velocity feedback here
        TrajectoryFollower controller = new TrajectoryFollower(
                logger, 2.4, 2.4, 0.0, 0.0);
        TrajectoryGenerator100 generator = new TrajectoryGenerator100();
        Map<String, Trajectory100> trajectories = generator.getTrajectorySet().getAllTrajectories();

        for (Map.Entry<String, Trajectory100> entry : trajectories.entrySet()) {
            Trajectory100 traj = entry.getValue();
            assertFalse(traj.isEmpty());
            TrajectoryTimeIterator traj_iterator = new TrajectoryTimeIterator(
                    new TrajectoryTimeSampler(traj));
            controller.setTrajectory(traj_iterator);
            final Pose2d kInjectedError = new Pose2d(0.3, -0.1, Rotation2d.fromDegrees(9.0));
            final FieldRelativeVelocity kInjectedVelocityError = new FieldRelativeVelocity(0.1, 0.3, 0.0);
            final double kInjectionTime = 20.0;
            Pose2d pose = traj.getPoint(0).state().state().getPose();
            FieldRelativeVelocity velocity = FieldRelativeVelocity.zero();
            SwerveSetpoint setpoint = null;
            double time = 0.0;
            double mDt = 0.005;
            boolean error_injected = false;
            while (!controller.isDone()) {
                if (!error_injected && time >= kInjectionTime) {
                    pose = GeometryUtil.transformBy(pose, kInjectedError);
                    velocity = velocity.plus(kInjectedVelocityError);
                    error_injected = true;
                }
                FieldRelativeVelocity speeds = controller.update(time, new SwerveModel(pose, velocity));
                if (true) {// setpoint == null) {
                    // Initialize from first chassis speeds.
                    ChassisSpeeds cs = ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds.x(), speeds.y(), speeds.theta(), pose.getRotation());
                    SwerveModuleStates states = kSmoothKinematicLimits.toSwerveModuleStates(
                            cs, velocity.theta());
                    setpoint = new SwerveSetpoint(cs, states);
                }

                // Don't use a twist here (assume Drive compensates for that)
                Pose2d delta = new Pose2d(
                        new Translation2d(
                                setpoint.getChassisSpeeds().vxMetersPerSecond * mDt,
                                setpoint.getChassisSpeeds().vyMetersPerSecond * mDt),
                        Rotation2d.fromRadians(setpoint.getChassisSpeeds().omegaRadiansPerSecond * mDt));
                pose = GeometryUtil.transformBy(pose, delta);
                // velocity = setpoint.getChassisSpeeds();
                velocity = speeds;

                // Inches and degrees
                Pose2d error = GeometryUtil.transformBy(GeometryUtil.inverse(pose),
                        controller.getSetpoint(time).get().state().getPose());

                assertEquals(0.0, error.getTranslation().getX(), 0.5);
                assertEquals(0.0, error.getTranslation().getY(), 0.5);
                assertEquals(0.0, error.getRotation().getDegrees(), 30.0);

                time += mDt;
            }
        }
    }
}