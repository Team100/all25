package org.team100.lib.follower;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class DrivePIDControllerTest {
    boolean dump = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest();

    @Test
    void testErrZero() {
        SwerveModel measurement = new SwerveModel();
        TimedPose setpoint = new TimedPose(
                new Pose2dWithMotion(new Pose2d()), 0, 0, 0);
        TrajectoryFollower controller = new TrajectoryFollower(logger, 2.4, 2.4, 0.0, 0.0);
        FieldRelativeDelta err = controller.positionError(measurement.pose(), setpoint);
        assertEquals(0, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXAhead() {
        SwerveModel measurement = new SwerveModel(new Pose2d(1, 0, new Rotation2d()));
        TimedPose setpoint = new TimedPose(
                new Pose2dWithMotion(new Pose2d()), 0, 0, 0);
        TrajectoryFollower controller = new TrajectoryFollower(logger, 2.4, 2.4, 0.0, 0.0);
        FieldRelativeDelta err = controller.positionError(measurement.pose(), setpoint);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXBehind() {
        SwerveModel measurement = new SwerveModel(new Pose2d(0, 0, new Rotation2d()));
        TimedPose setpoint = new TimedPose(
                new Pose2dWithMotion(new Pose2d(1, 0, new Rotation2d())), 0, 0, 0);
        TrajectoryFollower controller = new TrajectoryFollower(logger, 2.4, 2.4, 0.0, 0.0);
        FieldRelativeDelta err = controller.positionError(measurement.pose(), setpoint);
        assertEquals(1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    /** Rotation should not matter. */
    @Test
    void testErrXAheadWithRotation() {
        SwerveModel measurement = new SwerveModel(new Pose2d(1, 0, new Rotation2d(1)));
        TimedPose setpoint = new TimedPose(
                new Pose2dWithMotion(new Pose2d(0, 0, new Rotation2d(1))), 0, 0, 0);
        TrajectoryFollower controller = new TrajectoryFollower(logger, 2.4, 2.4, 0.0, 0.0);
        FieldRelativeDelta err = controller.positionError(measurement.pose(), setpoint);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testFieldRelativePIDControl() {
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

        Trajectory100 trajectory = TrajectoryPlanner.restToRest(
                waypoints,
                headings,
                constraints);

        // why is this so large?
        assertEquals(1300, trajectory.length());

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(trajectory);
        // note no velocity feedback here.
        TrajectoryFollower controller = new TrajectoryFollower(logger, 2.4, 2.4, 0.0, 0.0);
        controller.setTrajectory(iter);

        // this is a series of perfect trajectory following states,
        // based on the trajectory itself.

        {
            FieldRelativeVelocity output = controller.update(0,
                    new SwerveModel(new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(1.57079632679)),
                            FieldRelativeVelocity.zero()));
            verify(0, 0, 0, output);
        }

        {
            Pose2d measurement = new Pose2d(new Translation2d(0.25, -3.5), Rotation2d.fromRadians(1.69));
            FieldRelativeVelocity output = controller.update(
                    4.0, new SwerveModel(measurement, FieldRelativeVelocity.zero()));
            // remember, facing +90, moving -90, so this should be like -1
            // turning slowly to the left
            // +x -y
            verify(0.167, -0.850, 0.064, output);

            TimedPose path_setpoint = controller.getSetpoint(4).get();
            assertEquals(0.24, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-3.5, path_setpoint.state().getPose().getY(), 0.05);
            assertEquals(1.69, path_setpoint.state().getHeading().getRadians(), 0.01);
            assertEquals(4, path_setpoint.getTimeS(), 0.01);
            assertEquals(0.979, path_setpoint.velocityM_S(), 0.01);
            assertEquals(-0.008, path_setpoint.acceleration(), 0.001);

            FieldRelativeDelta positionError = controller.positionError(measurement, path_setpoint);
            assertEquals(0, positionError.getX(), 0.05);
            assertEquals(0, positionError.getY(), 0.05);
            assertEquals(0, positionError.getRadians(), 0.05);
        }
        {
            // this is the setpoint, so feedback shouldn't do anything.
            Pose2d measurement = new Pose2d(new Translation2d(1.74, -6.97), Rotation2d.fromRadians(2.22));
            FieldRelativeVelocity output = controller.update(8.0,
                    new SwerveModel(measurement, FieldRelativeVelocity.zero()));
            verify(0.592, -0.747, 0.098, output);

            TimedPose path_setpoint = controller.getSetpoint(8).get();
            assertEquals(1.74, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-6.97, path_setpoint.state().getPose().getY(), 0.01);
            assertEquals(2.19, path_setpoint.state().getHeading().getRadians(), 0.01);
            assertEquals(8, path_setpoint.getTimeS(), 0.001);
            assertEquals(0.955, path_setpoint.velocityM_S(), 0.001);
            assertEquals(0, path_setpoint.acceleration(), 0.001);

            FieldRelativeDelta positionError = controller.positionError(measurement, path_setpoint);
            assertEquals(0.00, positionError.getX(), 0.01);
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
