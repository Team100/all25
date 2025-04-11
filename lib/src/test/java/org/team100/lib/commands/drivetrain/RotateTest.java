package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

class RotateTest extends Fixtured implements Timeless {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.02;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private void verify(double speed, double angle) {
        assertEquals(speed, fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().speedMetersPerSecond(),
                kDelta);
        assertEquals(angle, fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().angle().get().getRadians(),
                kDelta);
    }

    private void print() {
        if (DEBUG)
            Util.printf("%6.3f %6.3f\n",
                    fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().speedMetersPerSecond(),
                    fixture.drive.getSwerveLocal().getDesiredStates().frontLeft().angle().get().getRadians());
    }

    /** This test is affected by the "Cross Track Error" thing. */
    @Test
    void testRotate() {
        fixture.collection.reset();
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, false);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // remember the test rotation rate is *very* slow.
        assertEquals(2.828, swerveKinodynamics.getMaxAngleSpeedRad_S(), 0.001);

        double targetAngle = Math.PI / 2;
        SwerveController controller = SwerveControllerFactory.test(logger);
        Rotate rotate = new Rotate(
                fixture.drive,
                controller,
                swerveKinodynamics,
                targetAngle);

        rotate.initialize();

        verify(0, 0);
        print();

        // the "cross track error" thing does slow it slightly.
        for (int i = 0; i < 12; ++i) {
            stepTime();
            fixture.drive.periodic();
            rotate.execute();
            print();
        }
        // going pretty fast
        verify(-0.417, -Math.PI / 4);

        for (int i = 0; i < 25; ++i) {
            stepTime();
            fixture.drive.periodic();
            rotate.execute();
            print();
        }
        // still going
        verify(-0.512, -Math.PI / 4);

        for (int i = 0; i < 36; ++i) {
            stepTime();
            fixture.drive.periodic();
            rotate.execute();
            print();
        }

        // done
        verify(0, -Math.PI / 4);

        // note we say we're done slightly before we actually are due to the tolerance,
        // which is fine.
        assertTrue(rotate.isFinished());

        rotate.end(false);
        verify(0, -Math.PI / 4);
    }
}
