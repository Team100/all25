package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.follower.DriveTrajectoryFollowerUtil;
import org.team100.lib.follower.FieldRelativeDrivePIDFFollower;
import org.team100.lib.follower.FieldRelativeDriveTrajectoryFollower;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.visualization.TrajectoryVisualization;

/**
 * These just exercise the code, they don't really test anything.
 */
class DriveToWaypoint100Test extends Fixtured {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    @Test
    void testWithPID() {
        DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(logger);
        DriveTrajectoryFollowerFactory driveControllerFactory = new DriveTrajectoryFollowerFactory(util);
        FieldRelativeDrivePIDFFollower.Log PIDFlog = new FieldRelativeDrivePIDFFollower.Log(logger);

        FieldRelativeDriveTrajectoryFollower controller = driveControllerFactory.testFieldRelativePIDF(PIDFlog);
        // the trajectory here should be a no-op.
        DriveToWaypoint100 command = new DriveToWaypoint100(
                logger,
                GeometryUtil.kPoseZero,
                fixture.drive,
                controller,
                fixture.swerveKinodynamics,
                0,
                viz);
        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        command.execute();
        command.end(false);
    }

    @Test
    void testWithFF() {
        DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(logger);
        DriveTrajectoryFollowerFactory driveControllerFactory = new DriveTrajectoryFollowerFactory(util);
        FieldRelativeDrivePIDFFollower.Log PIDFlog = new FieldRelativeDrivePIDFFollower.Log(logger);
        FieldRelativeDriveTrajectoryFollower controller = driveControllerFactory.testFieldRelativeFFOnly(PIDFlog);
        // the trajectory here should be a no-op.
        DriveToWaypoint100 command = new DriveToWaypoint100(
                logger,
                GeometryUtil.kPoseZero,
                fixture.drive,
                controller,
                fixture.swerveKinodynamics,
                0,
                viz);
        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        command.execute();
        command.end(false);
    }
}
