package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedCapsizeLimiterTest {
    private static final double kDelta = 0.001;

    /**
     * initial = target => zero delta v => no constraint
     */
    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        ChassisSpeedCapsizeLimiter c = new ChassisSpeedCapsizeLimiter(l);
        ChassisSpeeds result = c.limit(new ChassisSpeeds(), new ChassisSpeeds());
        assertEquals(0, result.vxMetersPerSecond, kDelta);
        assertEquals(0, result.vyMetersPerSecond, kDelta);
        assertEquals(0, result.omegaRadiansPerSecond, kDelta);
    }

    /**
     * initial (1,0) target (0,1) => delta v is 1.414 m/s.
     * accel is ~70, way over the limit of around 8
     * allowed deltav in 0.02 is 0.163, so the resulting speed should
     * be quite close to the initial value.
     */
    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        assertEquals(8.166, l.getMaxCapsizeAccelM_S2(), kDelta);
        ChassisSpeedCapsizeLimiter c = new ChassisSpeedCapsizeLimiter(l);
        ChassisSpeeds result = c.limit(
                new ChassisSpeeds(1, 0, 0),
                new ChassisSpeeds(0, 1, 0));
        assertEquals(0.884, result.vxMetersPerSecond, kDelta);
        assertEquals(0.115, result.vyMetersPerSecond, kDelta);
        assertEquals(0, result.omegaRadiansPerSecond, kDelta);
    }
}
