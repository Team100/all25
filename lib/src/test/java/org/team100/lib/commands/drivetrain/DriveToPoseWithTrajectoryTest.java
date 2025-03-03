package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.Target;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class DriveToPoseWithTrajectoryTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

    @Test
    void testSimple() {
        Pose2d goal = Pose2d.kZero;
        SwerveDriveSubsystem drive = fixture.drive;

        SwerveController controller = SwerveControllerFactory.test(logger);
        DriveToPoseWithTrajectory command = new DriveToPoseWithTrajectory(
                () -> goal,
                drive,
                (start, end) -> new Trajectory100(
                        List.of(new TimedPose(new Pose2dWithMotion(
                                Pose2d.kZero, new Pose2dWithMotion.MotionDirection(0, 0, 0), 0, 0), 0, 0, 0))),
                controller,
                viz);
        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        command.execute();
        command.end(false);
    }

    /** Demonstrate how to use DriveToWaypoint to go to apriltags. */
    @Test
    void testAprilTag() throws IOException {
        SwerveDriveSubsystem drive = fixture.drive;
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        Transform2d transform = new Transform2d(
                new Translation2d(-1, -1),
                Rotation2d.kZero);

        Optional<Pose2d> optGoal = Target.goal(layout, Alliance.Blue, 1, transform);
        assertTrue(optGoal.isPresent());
        Pose2d goal = optGoal.get();
        assertEquals(15.300, goal.getX(), kDelta);
        assertEquals(0.876, goal.getY(), kDelta);
        assertEquals(-0.942, goal.getRotation().getRadians(), kDelta);

        SwerveController m_controller = SwerveControllerFactory.test(logger);
        DriveToPoseWithTrajectory command = new DriveToPoseWithTrajectory(
                () -> goal, drive,
                (start, end) -> planner.movingToRest(start, end),
                m_controller, viz);
        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        command.execute();
        command.end(false);
    }

}
