package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;
import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.util.Util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedLimiterTest {
    private static final double kDelta = 0.001;

    @Test
    void testAnalyticDesaturation() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        ChassisSpeedLimiter l = new ChassisSpeedLimiter(limit);
        double maxV = k.getMaxDriveVelocityM_S();
        double maxOmega = k.getMaxAngleSpeedRad_S();
        assertEquals(5, maxV, kDelta);
        assertEquals(14.142, maxOmega, kDelta);
        // same cases as above

        {
            ChassisSpeeds s = new ChassisSpeeds(4, 0, 0);
            ChassisSpeeds i = l.proportional(s);
            assertEquals(4, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(0, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(6, 0, 0);
            ChassisSpeeds i = l.proportional(s);
            assertEquals(5, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(0, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(0, 0, 11.313);
            ChassisSpeeds i = l.proportional(s);
            assertEquals(0, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(11.313, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(0, 0, 12);
            ChassisSpeeds i = l.proportional(s);
            assertEquals(0, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(12, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAnalyticDesaturation2() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        ChassisSpeedLimiter l = new ChassisSpeedLimiter(limit);

        {
            ChassisSpeeds s = new ChassisSpeeds(2, 0, 5.656);
            ChassisSpeeds i = l.proportional(s);
            assertEquals(2, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(5.656, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(1.414, 1.414, 5.656);
            ChassisSpeeds i = l.proportional(s);
            assertEquals(1.414, i.vxMetersPerSecond, kDelta);
            assertEquals(1.414, i.vyMetersPerSecond, kDelta);
            assertEquals(5.656, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(2.828, 2.828, 14.142);
            ChassisSpeeds i = l.proportional(s);
            assertEquals(1.571, i.vxMetersPerSecond, kDelta);
            assertEquals(1.571, i.vyMetersPerSecond, kDelta);
            assertEquals(7.857, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAnalyticDesaturation3() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        ChassisSpeedLimiter l = new ChassisSpeedLimiter(limit);

        {
            ChassisSpeeds s = new ChassisSpeeds(2.828, 2.828, 7.05);
            ChassisSpeeds i = l.proportional(s);
            assertEquals(2.178, i.vxMetersPerSecond, kDelta);
            assertEquals(2.178, i.vyMetersPerSecond, kDelta);
            assertEquals(5.430, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAFewCases() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        ChassisSpeedLimiter l = new ChassisSpeedLimiter(limit);
        double maxV = k.getMaxDriveVelocityM_S();
        double maxOmega = k.getMaxAngleSpeedRad_S();
        assertEquals(5, maxV, kDelta);
        assertEquals(14.142, maxOmega, kDelta);
        {
            // the other way slows down more because it is pessimistic about theta.
            ChassisSpeeds s = new ChassisSpeeds(0.13, -1.95, -9.38);
            ChassisSpeeds i = l.proportional(s);
            assertEquals(0.123, i.vxMetersPerSecond, kDelta);
            assertEquals(-1.850, i.vyMetersPerSecond, kDelta);
            assertEquals(-8.898, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testEquivalentDesaturation() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        ChassisSpeedLimiter l = new ChassisSpeedLimiter(limit);
        double maxV = k.getMaxDriveVelocityM_S();
        double maxOmega = k.getMaxAngleSpeedRad_S();
        assertEquals(5, maxV, kDelta);
        assertEquals(14.142, maxOmega, kDelta);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            ChassisSpeeds s = new ChassisSpeeds(
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10);
            SwerveModuleStates ms = k.toSwerveModuleStates(s);
            // takes theta into account, can go faster sometimes
            ChassisSpeeds i1 = k.toChassisSpeedsWithDiscretization(ms, 0.02);
            // does not take theta into account
            ChassisSpeeds i2 = l.proportional(s);
            // i2 should never be faster
            double x2 = Math.abs(i2.vxMetersPerSecond);
            double x1 = Math.abs(i1.vxMetersPerSecond);
            if (x2 > x1 + 1e-6) {
                // Util.printf("X high %.8f %.8f\n", x2, x1);
                dump(i, s, i1, i2);
            }
            // but i1 shouldn't be *too* much faster.
            if ((x2 - x1) / x1 > 0.1) {
                // Util.printf("X low %.8f %.8f\n", x2, x1);
                dump(i, s, i1, i2);
            }
            double y2 = Math.abs(i2.vyMetersPerSecond);
            double y1 = Math.abs(i1.vyMetersPerSecond);
            if (y2 > y1 + 1e-6) {
                // Util.printf("Y high %.8f %.8f\n", y2, y1);
                dump(i, s, i1, i2);
            }
            if ((y2 - y1) / y1 > 0.1) {
                // Util.printf("Y low %.8f %.8f\n", y2, y1);
                dump(i, s, i1, i2);
            }
            double o2 = Math.abs(i2.omegaRadiansPerSecond);
            double o1 = Math.abs(i1.omegaRadiansPerSecond);
            if (o2 > o1 + 1e-6) {
                // Util.printf("omega high %.8f %.8f\n", o2, o1);
                dump(i, s, i1, i2);
            }
            if ((o2 - o1) / o1 > 0.1) {
                // Util.printf("omega low %.8f %.8f\n", o2, o1);
                dump(i, s, i1, i2);
            }
        }
    }

    @Test
    void testEquivalentDesaturationTwist() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        ChassisSpeedLimiter l = new ChassisSpeedLimiter(limit);
        double maxV = k.getMaxDriveVelocityM_S();
        double maxOmega = k.getMaxAngleSpeedRad_S();
        assertEquals(5, maxV, kDelta);
        assertEquals(14.142, maxOmega, kDelta);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            ChassisSpeeds s = new ChassisSpeeds(
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10);
            SwerveModuleStates ms = k.toSwerveModuleStates(s);
            // takes theta into account, can go faster sometimes
            ChassisSpeeds i1 = k.toChassisSpeedsWithDiscretization(ms, 0.02);
            // does not take theta into account
            ChassisSpeeds i2 = l.proportional(s);
            // i2 should never be faster
            double x2 = Math.abs(i2.vxMetersPerSecond);
            double x1 = Math.abs(i1.vxMetersPerSecond);
            if (x2 > x1 + 1e-6) {
                Util.printf("X high %.8f %.8f\n", x2, x1);
                dump(i, s, i1, i2);
            }
            // but i1 shouldn't be *too* much faster.
            if ((x2 - x1) / x1 > 0.1) {
                Util.printf("X low %.8f %.8f\n", x2, x1);
                dump(i, s, i1, i2);
            }
            double y2 = Math.abs(i2.vyMetersPerSecond);
            double y1 = Math.abs(i1.vyMetersPerSecond);
            if (y2 > y1 + 1e-6) {
                Util.printf("Y high %.8f %.8f\n", y2, y1);
                dump(i, s, i1, i2);
            }
            if ((y2 - y1) / y1 > 0.1) {
                Util.printf("Y low %.8f %.8f\n", y2, y1);
                dump(i, s, i1, i2);
            }
            double o2 = Math.abs(i2.omegaRadiansPerSecond);
            double o1 = Math.abs(i1.omegaRadiansPerSecond);
            if (o2 > o1 + 1e-6) {
                Util.printf("omega high %.8f %.8f\n", o2, o1);
                dump(i, s, i1, i2);
            }
            if ((o2 - o1) / o1 > 0.1) {
                Util.printf("omega low %.8f %.8f\n", o2, o1);
                dump(i, s, i1, i2);
            }
        }
    }

    /** shouldn't allow any movement at 6v. */
    @Test
    void testBrownout() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 6);
        ChassisSpeedLimiter l = new ChassisSpeedLimiter(limit);
        ChassisSpeeds target = new ChassisSpeeds(1, 0, 0);
        ChassisSpeeds limited = l.limit(target);
        assertEquals(0, limited.vxMetersPerSecond, kDelta);
        assertEquals(0, limited.vyMetersPerSecond, kDelta);
        assertEquals(0, limited.omegaRadiansPerSecond, kDelta);
    }

    private void dump(int i, ChassisSpeeds s, ChassisSpeeds i1, ChassisSpeeds i2) {
        Util.printf("%d -- IN: %s OUT1: %s OUT2: %s\n", i, s, i1, i2);
    }

}
