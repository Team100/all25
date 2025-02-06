package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.follower.DriveTrajectoryFollowerUtil;
import org.team100.lib.follower.FieldRelativeDrivePIDFFollower;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

class FancyTrajectoryTest extends Fixtured {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest();
        SwerveDriveSubsystem drive = fixture.drive;
        DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(logger);
        DriveTrajectoryFollowerFactory driveControllerFactory = new DriveTrajectoryFollowerFactory(util);
        FieldRelativeDrivePIDFFollower.Log PIDFlog = new FieldRelativeDrivePIDFFollower.Log(logger);
        FancyTrajectory command = new FancyTrajectory(
                logger,
                drive,
                driveControllerFactory.fieldRelativeFancyPIDF(PIDFlog),
                kSmoothKinematicLimits);
        command.initialize();
        command.execute();

        assertEquals(0, drive.getChassisSpeeds().vxMetersPerSecond, 0.001);
        assertEquals(0, drive.getChassisSpeeds().vyMetersPerSecond, 0.001);
        assertEquals(0, drive.getChassisSpeeds().omegaRadiansPerSecond, 0.001);

        command.end(false);
    }
}
