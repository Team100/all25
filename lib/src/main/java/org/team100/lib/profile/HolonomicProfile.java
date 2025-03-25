package org.team100.lib.profile;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveControl;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.Profile100.ResultWithETA;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

/**
 * Coordinates three axes so that their profiles complete at about the same
 * time, by adjusting the maximum allowed acceleration.
 * 
 * Note that because acceleration is adjusted, but not cruise velocity, the
 * resulting paths will not be straight, for rest-to-rest profiles.
 */
public class HolonomicProfile {
    private enum Profile {
        WPI,
        P100,
        EXP
    }

    private static final Profile kProfile = Profile.P100;
    /** For testing */
    private static final boolean DEBUG = false;
    private static final double ETA_TOLERANCE = 0.02;
    private static final double kDt = TimedRobot100.LOOP_PERIOD_S;

    private final Profile100 px;
    private final Profile100 py;
    private final Profile100 ptheta;

    private Profile100 ppx;
    private Profile100 ppy;
    private Profile100 pptheta;

    private HolonomicProfile(
            Profile100 px,
            Profile100 py,
            Profile100 ptheta) {
        this.px = px;
        this.py = py;
        this.ptheta = ptheta;
        ppx = px;
        ppy = py;
        pptheta = ptheta;
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
        logger.child("HolonomicProfile").stringLogger(Level.TRACE, "profile").log(() -> kProfile.name());
        switch (kProfile) {
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
            double maxAngularVel,
            double maxAngularAccel) {
        return new HolonomicProfile(
                new TrapezoidProfileWPI(maxXYVel, maxXYAccel),
                new TrapezoidProfileWPI(maxXYVel, maxXYAccel),
                new TrapezoidProfileWPI(maxAngularVel, maxAngularAccel));
    }

    public static HolonomicProfile trapezoidal(
            double maxXYVel,
            double maxXYAccel,
            double xyTolerance,
            double maxAngularVel,
            double maxAngularAccel,
            double angularTolerance) {
        return new HolonomicProfile(
                new TrapezoidProfile100(maxXYVel, maxXYAccel, xyTolerance),
                new TrapezoidProfile100(maxXYVel, maxXYAccel, xyTolerance),
                new TrapezoidProfile100(maxAngularVel, maxAngularAccel, angularTolerance));
    }

    public static HolonomicProfile currentLimitedExponential(
            double maxXYVel,
            double limitedXYAccel,
            double stallXYAccel,
            double maxAngularVel,
            double limitedAlpha,
            double stallAlpha) {
        return new HolonomicProfile(
                new CurrentLimitedExponentialProfile(maxXYVel, limitedXYAccel, stallXYAccel),
                new CurrentLimitedExponentialProfile(maxXYVel, limitedXYAccel, stallXYAccel),
                new CurrentLimitedExponentialProfile(maxXYVel, limitedAlpha, stallAlpha));
    }

    /** Reset the scale factors. */
    public void solve(SwerveModel i, SwerveModel g) {
        // first find the max ETA
        if (DEBUG) {
            Util.printf("ix %s gx %s\n", i.x(), g.x());
        }
        ResultWithETA rx = px.calculateWithETA(kDt, i.x(), g.x());
        ResultWithETA ry = py.calculateWithETA(kDt, i.y(), g.y());
        ResultWithETA rtheta = ptheta.calculateWithETA(kDt, i.theta(), g.theta());
        if (DEBUG) {
            Util.printf("rx %.3f ry %.3f rtheta %.3f\n", rx.etaS(), ry.etaS(), rtheta.etaS());
        }
        double slowETA = rx.etaS();
        slowETA = Math.max(slowETA, ry.etaS());
        slowETA = Math.max(slowETA, rtheta.etaS());

        double sx = px.solve(kDt, i.x(), g.x(), slowETA, ETA_TOLERANCE);
        double sy = py.solve(kDt, i.y(), g.y(), slowETA, ETA_TOLERANCE);
        double stheta = ptheta.solve(kDt, i.theta(), g.theta(), slowETA, ETA_TOLERANCE);

        if (DEBUG) {
            Util.printf("sx %.3f sy %.3f stheta %.3f\n", sx, sy, stheta);
        }

        ppx = px.scale(sx);
        ppy = py.scale(sy);
        pptheta = ptheta.scale(stheta);
    }

    /** Compute the control for the end of the next time step */
    public SwerveControl calculate(SwerveModel i, SwerveModel g) {
        if (i == null || g == null) {
            // this can happen on startup when the initial state hasn't yet been defined,
            // but the memo refresher is trying to update the references.
            return SwerveControl.zero();
        }
        if (DEBUG) {
            Util.printf("initial %s goal %s\n", i, g);
        }
        Control100 stateX = ppx.calculate(kDt, i.x(), g.x());
        Control100 stateY = ppy.calculate(kDt, i.y(), g.y());
        // theta is periodic; choose a setpoint angle near the goal.
        Model100 theta = new Model100(
                MathUtil.angleModulus(i.theta().x() - g.theta().x()) + g.theta().x(),
                i.theta().v());
        Control100 stateTheta = pptheta.calculate(kDt, theta, g.theta());
        return new SwerveControl(stateX, stateY, stateTheta);
    }
}
