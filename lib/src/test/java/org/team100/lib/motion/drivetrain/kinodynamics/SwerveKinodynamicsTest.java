package org.team100.lib.motion.drivetrain.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveKinodynamicsTest {
    private static final double kDelta = 0.001;

    /** From field relative speed to robot relative speed to modules and back. */
    @Test
    void testRoundTripMotionless() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds instantaneous = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);
        SwerveModuleStates states = unlimited.toSwerveModuleStates(instantaneous, 0.02);
        ChassisSpeeds implied = unlimited.toChassisSpeedsWithDiscretization(states, 0.02);
        FieldRelativeVelocity result = SwerveKinodynamics.fromInstantaneousChassisSpeeds(implied, theta);
        assertEquals(0, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

    /** From field relative speed to robot relative speed to modules and back. */
    @Test
    void testRoundTripDriveAndSpin() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        FieldRelativeVelocity v = new FieldRelativeVelocity(5, 0, 25);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds instantaneous = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);
        SwerveModuleStates states = unlimited.toSwerveModuleStates(instantaneous, 0.02);
        ChassisSpeeds implied = unlimited.toChassisSpeedsWithDiscretization(states, 0.02);
        FieldRelativeVelocity result = SwerveKinodynamics.fromInstantaneousChassisSpeeds(implied, theta);
        assertEquals(5, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(25, result.theta(), kDelta);
    }

    @Test
    void testComputedValues() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveV = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(
                driveV, 1, 1, 1, 1, 20 * Math.PI, track, track, wheelbase, wheelbase / 2,
                1);
        assertEquals(1, k.getMaxDriveVelocityM_S(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.353, r, kDelta);

        double omega = driveV / r;
        assertEquals(2.828, omega, kDelta);
        assertEquals(2.828, k.getMaxAngleSpeedRad_S(), kDelta);
    }

    @Test
    void testComputedValues2() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveV = 4;
        SwerveKinodynamics k = new SwerveKinodynamics(
                driveV, 1, 1, 1, 1, 20 * Math.PI, track, track, wheelbase, wheelbase / 2,
                1);
        assertEquals(4, k.getMaxDriveVelocityM_S(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.353, r, kDelta);

        double omega = driveV / r;
        assertEquals(11.313, omega, kDelta);
        assertEquals(11.313, k.getMaxAngleSpeedRad_S(), kDelta);
    }

    @Test
    void testComputedValues3() {
        double track = 1;
        double wheelbase = 1;
        double driveV = 4;
        SwerveKinodynamics k = new SwerveKinodynamics(
                driveV, 1, 1, 1, 1, 20 * Math.PI, track, track, wheelbase,
                wheelbase / 2,
                1);
        assertEquals(4, k.getMaxDriveVelocityM_S(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.707, r, kDelta);

        double omega = driveV / r;
        assertEquals(5.656, omega, kDelta);
        assertEquals(5.656, k.getMaxAngleSpeedRad_S(), kDelta);
    }

    @Test
    void testComputedAngularAcceleration() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveA = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(
                1, 1, driveA, 1, 1, 20 * Math.PI, track, track, wheelbase, wheelbase / 2,
                1);
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.353, r, kDelta);

        double omegaDot = 12 * driveA * r / (track * track + wheelbase * wheelbase);

        assertEquals(8.485, omegaDot, kDelta);
        assertEquals(8.485, k.getMaxAngleAccelRad_S2(), kDelta);
    }

    @Test
    void testComputedAngularAcceleration2() {
        double track = 1;
        double wheelbase = 1;
        double driveA = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(
                1, 1, driveA, 1, 1, 20 * Math.PI, track, track, wheelbase, wheelbase / 2,
                1);
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.707, r, kDelta);

        double omegaDot = 12 * driveA * r / (track * track + wheelbase * wheelbase);

        // scales inverse with size
        assertEquals(4.242, omegaDot, kDelta);
        assertEquals(4.242, k.getMaxAngleAccelRad_S2(), kDelta);
    }

    @Test
    void testComputedCapsize() {
        double track = 1;
        double wheelbase = 1;
        double vcg = 0.3;
        SwerveKinodynamics k = new SwerveKinodynamics(
                1, 1, 1, 1, 1, 20 * Math.PI, track, track, wheelbase, wheelbase / 2, vcg);
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), kDelta);

        double fulcrum = Math.min(track / 2, wheelbase / 2);
        assertEquals(0.5, fulcrum, kDelta);

        double accel = 9.8 * fulcrum / vcg;

        assertEquals(16.333, accel, kDelta);
        assertEquals(16.333, k.getMaxCapsizeAccelM_S2(), kDelta);
    }

    @Test
    void testAnalyticDesaturation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        double maxOmega = l.getMaxAngleSpeedRad_S();
        assertEquals(5, maxV, kDelta);
        assertEquals(14.142, maxOmega, kDelta);
        // same cases as above

        {
            ChassisSpeeds s = new ChassisSpeeds(4, 0, 0);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(4, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(0, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(6, 0, 0);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(5, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(0, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(0, 0, 11.313);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(0, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(11.313, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(0, 0, 12);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(0, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(12, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAnalyticDesaturation2() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();

        {
            ChassisSpeeds s = new ChassisSpeeds(2, 0, 5.656);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(2, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(5.656, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(1.414, 1.414, 5.656);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(1.414, i.vxMetersPerSecond, kDelta);
            assertEquals(1.414, i.vyMetersPerSecond, kDelta);
            assertEquals(5.656, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(2.828, 2.828, 14.142);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(1.571, i.vxMetersPerSecond, kDelta);
            assertEquals(1.571, i.vyMetersPerSecond, kDelta);
            assertEquals(7.857, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAnalyticDesaturation3() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();

        {
            ChassisSpeeds s = new ChassisSpeeds(2.828, 2.828, 7.05);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(2.178, i.vxMetersPerSecond, kDelta);
            assertEquals(2.178, i.vyMetersPerSecond, kDelta);
            assertEquals(5.430, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAnalyticDesaturation4() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        {
            // verify the sign of the omega clamp
            assertEquals(14.142, l.getMaxAngleSpeedRad_S(), kDelta);
            FieldRelativeVelocity s = new FieldRelativeVelocity(0, 0, -20);
            FieldRelativeVelocity i = l.analyticDesaturation(s);
            assertEquals(0, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(-14.142, i.theta(), kDelta);
        }
    }

    @Test
    void testAFewCases() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        double maxOmega = l.getMaxAngleSpeedRad_S();
        assertEquals(5, maxV, kDelta);
        assertEquals(14.142, maxOmega, kDelta);

        {
            // with no translation the wheel speed is ok
            ChassisSpeeds s = new ChassisSpeeds(0, 0, -9.38);
            SwerveModuleStates ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            assertEquals(3.316, ms.frontLeft().speedMetersPerSecond(), kDelta);
            assertEquals(3.316, ms.frontRight().speedMetersPerSecond(), kDelta);
            assertEquals(3.316, ms.rearLeft().speedMetersPerSecond(), kDelta);
            assertEquals(3.316, ms.rearRight().speedMetersPerSecond(), kDelta);
            // with an extra ~2m/s, it's too fast
            s = new ChassisSpeeds(0.13, -1.95, -9.38);
            ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            assertEquals(4.957, ms.frontLeft().speedMetersPerSecond(), kDelta);
            assertEquals(4.832, ms.frontRight().speedMetersPerSecond(), kDelta);
            assertEquals(2.506, ms.rearLeft().speedMetersPerSecond(), kDelta);
            assertEquals(2.250, ms.rearRight().speedMetersPerSecond(), kDelta);

            ChassisSpeeds i = l.toChassisSpeeds(ms);
            // so it slows down
            assertEquals(0.130, i.vxMetersPerSecond, kDelta);
            assertEquals(-1.95, i.vyMetersPerSecond, kDelta);
            assertEquals(-9.38, i.omegaRadiansPerSecond, kDelta);
        }
        {
            // the other way slows down more because it is pessimistic about theta.
            ChassisSpeeds s = new ChassisSpeeds(0.13, -1.95, -9.38);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(0.123, i.vxMetersPerSecond, kDelta);
            assertEquals(-1.850, i.vyMetersPerSecond, kDelta);
            assertEquals(-8.898, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testEquivalentDesaturation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        double maxOmega = l.getMaxAngleSpeedRad_S();
        assertEquals(5, maxV, kDelta);
        assertEquals(14.142, maxOmega, kDelta);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            ChassisSpeeds s = new ChassisSpeeds(
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10);
            SwerveModuleStates ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            // takes theta into account, can go faster sometimes
            ChassisSpeeds i1 = l.toChassisSpeeds(ms);
            // does not take theta into account
            ChassisSpeeds i2 = l.analyticDesaturation(s);
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
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        double maxOmega = l.getMaxAngleSpeedRad_S();
        assertEquals(5, maxV, kDelta);
        assertEquals(14.142, maxOmega, kDelta);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            ChassisSpeeds s = new ChassisSpeeds(
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10);
            SwerveModuleStates ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            // takes theta into account, can go faster sometimes
            ChassisSpeeds i1 = l.toChassisSpeeds(ms);
            // does not take theta into account
            ChassisSpeeds i2 = l.analyticDesaturation(s);
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

    @Test
    void testPreferRotation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        assertEquals(14.142, l.getMaxAngleSpeedRad_S(), kDelta);
        {
            // trivial case works
            FieldRelativeVelocity t = new FieldRelativeVelocity(0, 0, 0);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(0, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(0, i.theta(), kDelta);
        }
    }

    @Test
    void testPreferRotation2() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        assertEquals(14.142, l.getMaxAngleSpeedRad_S(), kDelta);
        {
            // inside the envelope => no change
            FieldRelativeVelocity t = new FieldRelativeVelocity(1, 0, 1);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(1, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(1, i.theta(), kDelta);
        }
        {
            // full v, half omega => half v
            FieldRelativeVelocity t = new FieldRelativeVelocity(5, 0, 7.05);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(2.507, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(7.05, i.theta(), kDelta);
        }
        {
            // full v, full omega => zero v, sorry
            FieldRelativeVelocity t = new FieldRelativeVelocity(5, 0, 14.142);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(0, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(14.142, i.theta(), kDelta);
        }
    }

    @Test
    void testDiscretizationNoEffect() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        // for this test the gyro rate and the commanded omega are the same,
        // though this is definitely not true in general
        {
            // pure rotation involves no discretization effect
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(0, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(0, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(1, impliedSpeeds.omegaRadiansPerSecond, kDelta);
        }
        {
            // pure translation involves no discretization effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(1, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(0, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(0, impliedSpeeds.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testDiscretizationWithEffect() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        // for this test the gyro rate and the commanded omega are the same,
        // though this is definitely not true in general
        {
            // holonomic does have discretization effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 1);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(0.999, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(-0.035, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(1, impliedSpeeds.omegaRadiansPerSecond, kDelta);

            // invert the discretization to extract the original speeds
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(states, 0.02);
            assertEquals(0.999, correctedImplied.vxMetersPerSecond, kDelta);
            assertEquals(0, correctedImplied.vyMetersPerSecond, kDelta);
            assertEquals(1, correctedImplied.omegaRadiansPerSecond, kDelta);
        }
        {
            // more spinning => bigger effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 3);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(0.994, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(-0.105, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(3, impliedSpeeds.omegaRadiansPerSecond, kDelta);

            // invert the discretization to extract the original speeds.
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(states, 0.02);
            assertEquals(1, correctedImplied.vxMetersPerSecond, kDelta);
            assertEquals(0, correctedImplied.vyMetersPerSecond, kDelta);
            assertEquals(3, correctedImplied.omegaRadiansPerSecond, kDelta);
        }
        {
            // longer time interval => bigger effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 3);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.2);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(0.944, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(-0.372, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(3, impliedSpeeds.omegaRadiansPerSecond, kDelta);

            // invert the discretization to extract the original speeds.
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(states, 0.2);
            assertEquals(1, correctedImplied.vxMetersPerSecond, kDelta);
            assertEquals(0, correctedImplied.vyMetersPerSecond, kDelta);
            assertEquals(3, correctedImplied.omegaRadiansPerSecond, kDelta);
        }
        {
            // longer time interval => bigger effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 3);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.2);
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(states, 0.2);
            assertEquals(1, correctedImplied.vxMetersPerSecond, kDelta);
            assertEquals(0, correctedImplied.vyMetersPerSecond, kDelta);
            assertEquals(3, correctedImplied.omegaRadiansPerSecond, kDelta);
        }
    }

    private void dump(int i, ChassisSpeeds s, ChassisSpeeds i1, ChassisSpeeds i2) {
        Util.printf("%d -- IN: %s OUT1: %s OUT2: %s\n", i, s, i1, i2);
    }

}
