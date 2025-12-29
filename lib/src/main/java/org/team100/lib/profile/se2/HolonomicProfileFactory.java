package org.team100.lib.profile.se2;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.r1.CurrentLimitedExponentialProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidProfileWPI;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

public class HolonomicProfileFactory {
    private static final Profile PROFILE = Profile.P100;

    private enum Profile {
        WPI,
        P100,
        EXP
    }

    /**
     * Make a holonomic profile using the kinodynamic absolute maxima, scaled as
     * specified.
     */
    public static HolonomicProfile get(
            LoggerFactory logger,
            SwerveKinodynamics kinodynamics,
            double vScale,
            double aScale,
            double omegaScale,
            double alphaScale) {
        LoggerFactory log = logger.name("HolonomicProfile");
        log.stringLogger(Level.TRACE, "profile").log(() -> PROFILE.name());
        switch (PROFILE) {
            case EXP -> {
                return currentLimitedExponential(
                        kinodynamics.getMaxDriveVelocityM_S() * vScale,
                        kinodynamics.getMaxDriveAccelerationM_S2() * aScale,
                        kinodynamics.getStallAccelerationM_S2() * aScale,
                        kinodynamics.getMaxAngleSpeedRad_S() * omegaScale,
                        kinodynamics.getMaxAngleAccelRad_S2() * alphaScale,
                        kinodynamics.getMaxAngleStallAccelRad_S2() * alphaScale);
            }

            case WPI -> {
                return wpi(
                        kinodynamics.getMaxDriveVelocityM_S() * vScale,
                        kinodynamics.getMaxDriveAccelerationM_S2() * aScale,
                        kinodynamics.getMaxAngleSpeedRad_S() * omegaScale,
                        kinodynamics.getMaxAngleAccelRad_S2() * alphaScale);
            }

            case P100 -> {
                return trapezoidal(
                        log,
                        kinodynamics.getMaxDriveVelocityM_S() * vScale,
                        kinodynamics.getMaxDriveAccelerationM_S2() * aScale,
                        0.05,
                        kinodynamics.getMaxAngleSpeedRad_S() * omegaScale,
                        kinodynamics.getMaxAngleAccelRad_S2() * alphaScale,
                        0.1);
            }

            default -> {
                return null;
            }
        }
    }

    public static HolonomicProfile wpi(
            double maxXYVel,
            double maxXYAccel,
            double maxOmega,
            double maxAlpha) {
        return new HolonomicProfile(
                new TrapezoidProfileWPI(maxXYVel, maxXYAccel),
                new TrapezoidProfileWPI(maxXYVel, maxXYAccel),
                new TrapezoidProfileWPI(maxOmega, maxAlpha));
    }

    public static HolonomicProfile trapezoidal(
            LoggerFactory log,
            double maxXYVel,
            double maxXYAccel,
            double xyTolerance,
            double maxOmega,
            double maxAlpha,
            double angularTolerance) {
        return new HolonomicProfile(
                new TrapezoidIncrementalProfile(log.name("x"), maxXYVel, maxXYAccel, xyTolerance),
                new TrapezoidIncrementalProfile(log.name("y"), maxXYVel, maxXYAccel, xyTolerance),
                new TrapezoidIncrementalProfile(log.name("theta"), maxOmega, maxAlpha, angularTolerance));
    }

    public static HolonomicProfile currentLimitedExponential(
            double maxXYVel,
            double limitedXYAccel,
            double stallXYAccel,
            double maxOmega,
            double limitedAlpha,
            double stallAlpha) {
        return new HolonomicProfile(
                new CurrentLimitedExponentialProfile(maxXYVel, limitedXYAccel, stallXYAccel),
                new CurrentLimitedExponentialProfile(maxXYVel, limitedXYAccel, stallXYAccel),
                new CurrentLimitedExponentialProfile(maxOmega, limitedAlpha, stallAlpha));
    }

    public static FreeRotationProfile freeRotationCurrentLimitedExponential(
            double maxXYVel,
            double limitedXYAccel,
            double stallXYAccel,
            double maxOmega,
            double limitedAlpha,
            double stallAlpha) {
        return new FreeRotationProfile(
                new CurrentLimitedExponentialProfile(maxXYVel, limitedXYAccel, stallXYAccel),
                new CurrentLimitedExponentialProfile(maxXYVel, limitedXYAccel, stallXYAccel),
                new CurrentLimitedExponentialProfile(maxOmega, limitedAlpha, stallAlpha));
    }

    public static HolonomicProfile currentLimitedExponential(
            SwerveKinodynamics k, double xyScale, double rotScale) {
        return currentLimitedExponential(
                k.getMaxDriveVelocityM_S() * xyScale,
                k.getMaxDriveAccelerationM_S2() * xyScale,
                k.getStallAccelerationM_S2() * xyScale,
                k.getMaxAngleSpeedRad_S() * rotScale,
                k.getMaxAngleAccelRad_S2() * rotScale,
                k.getMaxAngleStallAccelRad_S2() * rotScale);
    }

    public static FreeRotationProfile freeRotationCurrentLimitedExponential(
            SwerveKinodynamics k, double xyScale, double rotScale) {
        return freeRotationCurrentLimitedExponential(
                k.getMaxDriveVelocityM_S() * xyScale,
                k.getMaxDriveAccelerationM_S2() * xyScale,
                k.getStallAccelerationM_S2() * xyScale,
                k.getMaxAngleSpeedRad_S() * rotScale,
                k.getMaxAngleAccelRad_S2() * rotScale,
                k.getMaxAngleStallAccelRad_S2() * rotScale);
    }
}
