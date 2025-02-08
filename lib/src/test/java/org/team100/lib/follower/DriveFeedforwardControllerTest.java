package org.team100.lib.follower;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class DriveFeedforwardControllerTest {
    boolean dump = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest();

    @Test
    void testFieldRelativeFeedforwardOnly() {

        // first right and then ahead
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(10, -10, Rotation2d.fromDegrees(0)));
        // face +y and end up -x
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));
        // so this trajectory is actually (robot-relative) -x the whole way, more or
        // less.

        List<TimingConstraint> constraints = new TimingConstraintFactory(kSmoothKinematicLimits).fast();

        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypoints, headings, constraints);
        assertEquals(1300, trajectory.length());

        TrajectoryTimeSampler view = new TrajectoryTimeSampler(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(view);
        TrajectoryFollower controller = new TrajectoryFollower(logger, 0, 0, 0, 0);
        controller.setTrajectory(iter);

        // this is a series of perfect trajectory following states,
        // based on the trajectory itself.

        {
            FieldRelativeVelocity output = controller.update(0,
                    new SwerveModel(
                            new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(1.57079632679)),
                            FieldRelativeVelocity.zero()));
            verify(0, 0, 0, output);
        }

        {
            Pose2d measurement = new Pose2d(new Translation2d(0.25, -3.5), Rotation2d.fromRadians(1.69));

            // note 4s is a *long* time. the max v is 1.0 so this will be cruising
            // also the trajectory is long, quarter circle with radius 10, so in 4
            // sec it will be like 1/3 the way around
            FieldRelativeVelocity output = controller.update(4.0,
                    new SwerveModel(measurement, FieldRelativeVelocity.zero()));

            // field relative this should be mostly -y but with some +x
            // verify(-0.976, -0.075, 0.075, output);
            verify(0.191, -0.961, 0.075, output);

            TimedPose path_setpoint = controller.getSetpoint(4).get();
            assertEquals(0.24, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-3.454, path_setpoint.state().getPose().getY(), 0.05);
            assertEquals(1.685, path_setpoint.state().getHeading().getRadians(), 0.01);
            assertEquals(4, path_setpoint.getTimeS(), 0.01);
            assertEquals(0.979, path_setpoint.velocityM_S(), 0.01);
            assertEquals(-0.008, path_setpoint.acceleration(), 0.001);

            FieldRelativeDelta positionError = controller.positionError(measurement, path_setpoint);
            assertEquals(0, positionError.getX(), 0.05);
            assertEquals(0, positionError.getY(), 0.05);
            assertEquals(0, positionError.getRadians(), 0.05);
        }
        {
            Pose2d measurement = new Pose2d(new Translation2d(1.74, -6.96), Rotation2d.fromRadians(2.22));
            FieldRelativeVelocity output = controller.update(8.0,
                    new SwerveModel(measurement, FieldRelativeVelocity.zero()));
            // at 8 s we're almost at the corner.
            verify(0.584, -0.756, 0.170, output);

            TimedPose path_setpoint = controller.getSetpoint(8).get();
            assertEquals(1.74, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-6.96, path_setpoint.state().getPose().getY(), 0.01);
            assertEquals(2.18, path_setpoint.state().getHeading().getRadians(), 0.01);
            assertEquals(8, path_setpoint.getTimeS(), 0.001);
            assertEquals(0.955, path_setpoint.velocityM_S(), 0.001);
            assertEquals(0, path_setpoint.acceleration(), 0.001);

            FieldRelativeDelta positionError = controller.positionError(measurement, path_setpoint);
            assertEquals(0, positionError.getX(), 0.01);
            assertEquals(0, positionError.getY(), 0.01);
            assertEquals(-0.03, positionError.getRadians(), 0.01);
        }
    }

    void verify(double vx, double vy, double omega, ChassisSpeeds output) {
        assertEquals(vx, output.vxMetersPerSecond, 0.05);
        assertEquals(vy, output.vyMetersPerSecond, 0.05);
        assertEquals(omega, output.omegaRadiansPerSecond, 0.05);
    }

    void verify(double vx, double vy, double omega, FieldRelativeVelocity output) {
        assertEquals(vx, output.x(), 0.001);
        assertEquals(vy, output.y(), 0.001);
        assertEquals(omega, output.theta(), 0.001);
    }
}
