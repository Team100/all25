package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.util.Util;

class SwerveUtilTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.001;

    @Test
    void testIsAccel() {
        // hard left turn is not accel
        assertFalse(SwerveUtil.isAccel(
                new FieldRelativeVelocity(1, 0, 0),
                new FieldRelativeVelocity(0, 1, 0)));
        // speed up veering left
        assertTrue(SwerveUtil.isAccel(
                new FieldRelativeVelocity(0.5, 0.5, 0),
                new FieldRelativeVelocity(0, 1, 0)));
    }

    @Test
    void testGetMaxVelStepWithVelocityDependentAccel() {
        // available acceleration is not always the max.
        // a motor without current limiting has a straight declining torque curve
        // a motor with current limiting has a constant torque curve for awhile
        // hm, how to get the motor model in here?
    }

    @Test
    void testGetAccelLimit() {
        // this is to figure out why the Oscillate test isn't returning
        // exactly the right result
        SwerveKinodynamics limits = new Fixture().swerveKinodynamics;
        assertEquals(1, limits.getMaxDriveAccelerationM_S2(), kDelta);
        double accelLimit = SwerveUtil.getAccelLimit(limits,
                new FieldRelativeVelocity(0.92, 0, 0),
                new FieldRelativeVelocity(0.94, 0, 0));
        assertEquals(0.8, accelLimit, kDelta);
    }

    @Test
    void testMinAccel() {
        // this is to figure out why the Oscillate test isn't returning
        // exactly the right result
        SwerveKinodynamics limits = new Fixture().swerveKinodynamics;
        // the test asks for 1 m/s/s
        assertEquals(1, limits.getMaxDriveAccelerationM_S2(), kDelta);
        // the problem is that the maximum possible velocity is right at the
        // maximum commanded velocity, so the motor can't execute the constant
        // accel command.
        assertEquals(1, limits.getMaxDriveVelocityM_S(), kDelta);
        assertEquals(10, limits.getStallAccelerationM_S2(), kDelta);
        // this returns 0.8 which is wrong
        double accelLimit = SwerveUtil.minAccel(limits, 0.92);
        assertEquals(0.8, accelLimit, kDelta);
    }

    @Test
    void simMinAccel() {
        // simulate full-throttle to see the exponential curve.
        // https://docs.google.com/spreadsheets/d/1k-g8_blQP3X1RNtjFQgk1CJXyzzvNLNuUbWhaNLduvw
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        double v = 0;
        final double dt = 0.02;
        for (double t = 0; t < 3; t += dt) {
            if (DEBUG)
                Util.printf("%5.3f %5.3f\n", t, v);
            double a = SwerveUtil.minAccel(limits, v);
            v += dt * a;
        }
    }

    @Test
    void testJerkLimit() {
        
    }
}
