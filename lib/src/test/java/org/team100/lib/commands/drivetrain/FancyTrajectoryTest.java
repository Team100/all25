package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.follower.TrajectoryFollowerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;

class FancyTrajectoryTest extends Fixtured implements Timeless {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest();
        SwerveDriveSubsystem drive = fixture.drive;
        FancyTrajectory command = new FancyTrajectory(
                logger,
                drive,
                TrajectoryFollowerFactory.fieldRelativeFancyPIDF(logger),
                kSmoothKinematicLimits);
        command.initialize();
        command.execute();

        assertEquals(0, drive.getChassisSpeeds().vxMetersPerSecond, 0.001);
        assertEquals(0, drive.getChassisSpeeds().vyMetersPerSecond, 0.001);
        assertEquals(0, drive.getChassisSpeeds().omegaRadiansPerSecond, 0.001);

        command.end(false);
    }
}
