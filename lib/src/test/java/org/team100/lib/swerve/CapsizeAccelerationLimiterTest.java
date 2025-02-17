package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

class CapsizeAccelerationLimiterTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    /** zero delta v => no constraint */
    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        CapsizeAccelerationLimiter c = new CapsizeAccelerationLimiter(logger, l);
        double s = c.enforceCentripetalLimit(new ChassisSpeeds(), new ChassisSpeeds());
        assertEquals(1, s, kDelta);
    }

    /**
     * total delta v is 1.414 m/s.
     */
    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        CapsizeAccelerationLimiter c = new CapsizeAccelerationLimiter(logger, l);
        double s = c.enforceCentripetalLimit(new ChassisSpeeds(1, 0, 0), new ChassisSpeeds(0, 1, 0));
        assertEquals(0.115, s, kDelta);
    }
}
