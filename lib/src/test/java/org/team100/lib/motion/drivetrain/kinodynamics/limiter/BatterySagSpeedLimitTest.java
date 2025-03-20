package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

public class BatterySagSpeedLimitTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private double volts;

    @Test
    void testLimit() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit s = new BatterySagSpeedLimit(logger, k, () -> volts);
        volts = 0;
        assertEquals(0, s.getMaxDriveVelocityM_S(), kDelta);
        volts = 6;
        assertEquals(0, s.getMaxDriveVelocityM_S(), kDelta);
        volts = 6.5;
        assertEquals(1.458, s.getMaxDriveVelocityM_S(), kDelta);
        volts = 7;
        assertEquals(2.917, s.getMaxDriveVelocityM_S(), kDelta);
        volts = 8;
        assertEquals(3.333, s.getMaxDriveVelocityM_S(), kDelta);
        volts = 12;
        assertEquals(5, s.getMaxDriveVelocityM_S(), kDelta);
    }

}
