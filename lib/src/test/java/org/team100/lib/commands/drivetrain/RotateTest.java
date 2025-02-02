package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.drivetrain.HolonomicDriveControllerFactory;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.sensors.MockGyro;
import org.team100.lib.testing.Timeless;

class RotateTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.02;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testRotate() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);
        SwerveDriveSubsystem swerveDriveSubsystem = fixture.drive;
        MockGyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // remember the test rotation rate is *very* slow.
        assertEquals(2.828, swerveKinodynamics.getMaxAngleSpeedRad_S(), 0.001);
        double targetAngle = Math.PI / 2;
        HolonomicFieldRelativeController controller = HolonomicDriveControllerFactory.get(
                logger,
                new HolonomicFieldRelativeController.Log(logger));
        Rotate rotate = new Rotate(
                logger,
                swerveDriveSubsystem,
                controller,
                gyro,
                swerveKinodynamics,
                targetAngle);

        rotate.initialize();

        assertEquals(0, rotate.refTheta.x(), kDelta); // at start
        assertEquals(0, fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().speedMetersPerSecond, kDelta);
        assertEquals(0, fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().angle.get().getRadians(), kDelta);

        // steering
        for (int i = 0; i < 13; ++i) {
            stepTime(0.02);
            fixture.drive.periodic();
            rotate.execute();
        }
        assertEquals(0, rotate.refTheta.x(), 0.01);
        // now we're ready to start rotating
        assertEquals(-0.02, fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().speedMetersPerSecond, kDelta);
        assertEquals(-Math.PI / 4,
                fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().angle.get().getRadians(), 0.1);

        // finished steering, start rotating
        for (int i = 0; i < 25; ++i) {
            stepTime(0.02);
            fixture.drive.periodic();
            rotate.execute();
        }
        assertEquals(0.461, rotate.refTheta.x(), 0.2);
        assertEquals(-0.512, fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().speedMetersPerSecond,
                kDelta);
        assertEquals(-Math.PI / 4,
                fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().angle.get().getRadians(), 0.1);

        // should be done rotating now
        for (int i = 0; i < 50; ++i) {
            stepTime(0.02);
            fixture.drive.periodic();
            rotate.execute();
        }

        assertEquals(Math.PI / 2, rotate.refTheta.x(), kDelta);
        // assertEquals(-0.403,
        // fixture.drive.desiredStates().frontLeft().speedMetersPerSecond,
        // kDelta);
        assertEquals(-Math.PI / 4,
                fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().angle.get().getRadians(),
                kDelta);

        for (int i = 0; i < 113; ++i) {
            stepTime(0.02);
            fixture.drive.periodic();
            rotate.execute();
        }

        assertTrue(rotate.isFinished());

        rotate.end(false);
        assertEquals(0, fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().speedMetersPerSecond, kDelta);
        assertEquals(-Math.PI / 4,
                fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().angle.get().getRadians(),
                kDelta);
    }
}
