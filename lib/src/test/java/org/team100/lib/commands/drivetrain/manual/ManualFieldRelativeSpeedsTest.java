package org.team100.lib.commands.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

class ManualFieldRelativeSpeedsTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testTwistZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(logger, limits);
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        SwerveModel s = new SwerveModel();
        FieldRelativeVelocity twist = manual.apply(s, input);
        assertEquals(0, twist.x(), kDelta);
        assertEquals(0, twist.y(), kDelta);
        assertEquals(0, twist.theta(), kDelta);
    }

    @Test
    void testTwistNonzero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(logger, limits);
        // these inputs are clipped
        DriverControl.Velocity input = new DriverControl.Velocity(1, 2, 3);
        SwerveModel s = new SwerveModel();
        FieldRelativeVelocity twist = manual.apply(s, input);
        assertEquals(0.447, twist.x(), kDelta);
        assertEquals(0.894, twist.y(), kDelta);
        assertEquals(2.828, twist.theta(), kDelta);
    }

}
