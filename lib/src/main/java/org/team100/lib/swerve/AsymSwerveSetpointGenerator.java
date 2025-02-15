package org.team100.lib.swerve;

import java.util.function.DoubleSupplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Just-in-time kinodynamic limits.
 * 
 * This version uses different limits for acceleration and for deceleration,
 * which better matches real robot behavior.
 * 
 * Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or
 * from a path follower), and outputs a new setpoint that respects all of the
 * kinematic constraints on module rotation speed and wheel velocity and
 * acceleration. By generating a new setpoint every iteration, the robot will
 * converge to the desired setpoint quickly while avoiding any intermediate
 * state that is kinematically infeasible (and can result in wheel slip or robot
 * heading drift as a result).
 */
public class AsymSwerveSetpointGenerator implements Glassy {
    private static final boolean DEBUG = true;
    // turns greater than this will flip
    // this used to be pi/2, which resulted in "square corner" paths
    private static final double flipLimitRad = 3 * Math.PI / 4;

    private final SwerveKinodynamics m_limits;

    private final CapsizeAccelerationLimiter m_centripetalLimiter;
    private final SteeringOverride m_SteeringOverride;
    private final SteeringRateLimiter m_steeringRateLimiter;
    private final DriveAccelerationLimiter m_DriveAccelerationLimiter;
    private final BatterySagLimiter m_BatterySagLimiter;

    public AsymSwerveSetpointGenerator(
            LoggerFactory parent,
            SwerveKinodynamics limits,
            DoubleSupplier batteryVoltage) {
        m_limits = limits;
        m_centripetalLimiter = new CapsizeAccelerationLimiter(parent, limits);
        m_SteeringOverride = new SteeringOverride(parent, limits);
        m_steeringRateLimiter = new SteeringRateLimiter(parent, limits);
        m_DriveAccelerationLimiter = new DriveAccelerationLimiter(parent, limits);
        m_BatterySagLimiter = new BatterySagLimiter(batteryVoltage);
    }

    /**
     * Generate a new setpoint.
     * 
     * TODO: this is supposed to discretize the speeds prior to inverse kinematics
     * but it seems not to.
     * 
     * or maybe it does?
     * 
     */
    public SwerveSetpoint generateSetpoint(
            SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredState) {
        if (DEBUG)
            Util.printf("desired %s\n", desiredState);
        double min_s = 1.0;

        SwerveModuleStates prevModuleStates = prevSetpoint.getModuleStates();
        // the desired module state speeds are always positive.
        SwerveModuleStates desiredModuleStates = m_limits.toSwerveModuleStates(desiredState);
        // desiredState = desaturate(desiredState, desiredModuleStates);

        if (GeometryUtil.isZero(desiredState)) {
            desiredModuleStates = prevModuleStates.motionless();
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prev_vx = computeVx(prevModuleStates);
        double[] prev_vy = computeVy(prevModuleStates);
        // elements may be null.
        Rotation2d[] prev_heading = computeHeading(prevModuleStates);

        double[] desired_vx = computeVx(desiredModuleStates);
        double[] desired_vy = computeVy(desiredModuleStates);
        // elements may be null.
        Rotation2d[] desired_heading = computeHeading(desiredModuleStates);

        boolean shouldStopAndReverse = shouldStopAndReverse(prev_heading, desired_heading);
        if (shouldStopAndReverse
                && !GeometryUtil.isZero(prevSetpoint.getChassisSpeeds())
                && !GeometryUtil.isZero(desiredState)) {
            // It will (likely) be faster to stop the robot, rotate the modules in place to
            // the complement of the desired angle, and accelerate again.
            return generateSetpoint(prevSetpoint, new ChassisSpeeds());
        }

        // Compute the deltas between start and goal. We can then interpolate from the
        // start state to the goal state; then
        // find the amount we can move from start towards goal in this cycle such that
        // no kinematic limit is exceeded.

        ChassisSpeeds chassisSpeeds = prevSetpoint.getChassisSpeeds();
        double dx = desiredState.vxMetersPerSecond - chassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - chassisSpeeds.vyMetersPerSecond;
        double dtheta = desiredState.omegaRadiansPerSecond - chassisSpeeds.omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1,
        // we are at desiredState.

        double centripetal_min_s = m_centripetalLimiter.enforceCentripetalLimit(dx, dy);
        if (DEBUG)
            Util.printf("centripetal %f\n", centripetal_min_s);

        min_s = Math.min(min_s, centripetal_min_s);

        // In cases where an individual module is stopped, we want to remember the right
        // steering angle to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically
        // lazy).
        SwerveModuleState100[] prevModuleStatesAll = prevModuleStates.all();
        Rotation2d[] overrideSteering = new Rotation2d[prevModuleStatesAll.length];

        if (GeometryUtil.isZero(desiredState)) {
            for (int i = 0; i < prevModuleStatesAll.length; ++i) {
                if (prevModuleStatesAll[i].angle().isEmpty()) {
                    overrideSteering[i] = null;
                } else {
                    overrideSteering[i] = prevModuleStatesAll[i].angle().get();
                }
            }
        } else {
            double override_min_s = m_SteeringOverride.overrideIfStopped(
                    desiredModuleStates,
                    prevModuleStates,
                    overrideSteering);
            if (DEBUG)
                Util.printf("override %f\n", override_min_s);
            min_s = Math.min(min_s, override_min_s);

            double steering_min_s = m_steeringRateLimiter.enforceSteeringLimit(
                    prev_vx,
                    prev_vy,
                    prev_heading,
                    desired_vx,
                    desired_vy,
                    desired_heading,
                    overrideSteering);
            if (DEBUG)
                Util.printf("steering %f\n", steering_min_s);
            min_s = Math.min(min_s, steering_min_s);
        }

        double accel_min_s = m_DriveAccelerationLimiter.enforceWheelAccelLimit(
                prev_vx,
                prev_vy,
                desired_vx,
                desired_vy);
        if (DEBUG)
            Util.printf("accel %f\n", accel_min_s);
        min_s = Math.min(min_s, accel_min_s);

        double battery_min_s = m_BatterySagLimiter.get();
        if (DEBUG)
            Util.printf("battery %f\n", battery_min_s);

        min_s = Math.min(min_s, battery_min_s);

        return makeSetpoint(
                prevSetpoint,
                prevModuleStates,
                dx,
                dy,
                dtheta,
                min_s,
                overrideSteering);
    }

    ///////////////////////////////////////////////////////

    private double[] computeVx(SwerveModuleStates states) {
        SwerveModuleState100[] statesAll = states.all();
        double[] vx = new double[statesAll.length];
        for (int i = 0; i < statesAll.length; ++i) {
            SwerveModuleState100 state = statesAll[i];
            if (Math.abs(state.speedMetersPerSecond()) < 1e-6 || state.angle().isEmpty()) {
                vx[i] = 0;
            } else {
                vx[i] = state.angle().get().getCos() * state.speedMetersPerSecond();
            }
        }
        return vx;
    }

    private double[] computeVy(SwerveModuleStates states) {
        SwerveModuleState100[] statesAll = states.all();
        double[] vy = new double[statesAll.length];
        for (int i = 0; i < statesAll.length; ++i) {
            SwerveModuleState100 state = statesAll[i];
            if (Math.abs(state.speedMetersPerSecond()) < 1e-6 || state.angle().isEmpty()) {
                vy[i] = 0;
            } else {
                vy[i] = state.angle().get().getSin() * state.speedMetersPerSecond();
            }
        }
        return vy;
    }

    /**
     * Which way each module is actually going, taking speed polarity into account.
     * 
     * @return elements are nullable.
     */
    private Rotation2d[] computeHeading(SwerveModuleStates states) {
        SwerveModuleState100[] statesAll = states.all();
        Rotation2d[] heading = new Rotation2d[statesAll.length];
        for (int i = 0; i < statesAll.length; ++i) {
            if (statesAll[i].angle().isEmpty()) {
                heading[i] = null;
                continue;
            }
            heading[i] = statesAll[i].angle().get();
            if (statesAll[i].speedMetersPerSecond() < 0.0) {
                heading[i] = GeometryUtil.flip(heading[i]);
            }
        }
        return heading;
    }

    /**
     * If we want to go back the way we came, it might be faster to stop
     * and then reverse. This is certainly true for near-180 degree turns, but
     * it's definitely not true for near-90 degree turns.
     */
    private boolean shouldStopAndReverse(Rotation2d[] prev_heading, Rotation2d[] desired_heading) {
        for (int i = 0; i < prev_heading.length; ++i) {
            if (desired_heading[i] == null || prev_heading[i] == null) {
                return false;
            }
            Rotation2d diff = desired_heading[i].minus(prev_heading[i]);
            if (Math.abs(diff.getRadians()) < flipLimitRad) {
                return false;
            }
        }
        return true;
    }

    /** States are discretized. */
    private SwerveSetpoint makeSetpoint(
            final SwerveSetpoint prevSetpoint,
            SwerveModuleStates prevModuleStates,
            double dx,
            double dy,
            double dtheta,
            double min_s,
            Rotation2d[] overrideSteering) {
        ChassisSpeeds setpointSpeeds = makeSpeeds(
                prevSetpoint.getChassisSpeeds(),
                dx,
                dy,
                dtheta,
                min_s);
        // the speeds in these states are always positive.
        // the kinematics produces an empty angle for zero speed, but here we
        // know the previous state so use that.
        SwerveModuleStates setpointStates = m_limits.toSwerveModuleStates(setpointSpeeds);
        setpointStates = setpointStates.overwriteEmpty(prevModuleStates);
        setpointStates = setpointStates.override(overrideSteering);
        setpointStates = setpointStates.flipIfRequired(prevModuleStates);

        // sync up the speeds with the states. is this needed?
        ChassisSpeeds finalSpeeds = m_limits.toChassisSpeedsWithDiscretization(
                setpointStates,
                TimedRobot100.LOOP_PERIOD_S);

        if (DEBUG)
            Util.printf("prev %s initial %s final %s\n",
                    prevSetpoint.getChassisSpeeds(), setpointSpeeds, finalSpeeds);
        return new SwerveSetpoint(finalSpeeds, setpointStates);
    }

    /**
     * Scales the commanded accelerations by min_s, i.e. applies the constraints
     * calculated earlier.
     * 
     * This used to also apply the rotation to the translation in a way that I think
     * was wrong.
     * 
     * 
     * * Applies two transforms to the previous speed:
     * 
     * 1. Scales the commanded accelerations by min_s, i.e. applies the constraints
     * calculated earlier.
     * 
     * 2. Transforms translations according to the rotational velocity, regardless
     * of
     * min_s -- essentially modeling inertia. This part was missing before, which I
     * think must just be a mistake.
     * 
     * 
     * why does this work?
     * 
     * we want to maintain the intended direction of motion in a field-relative way
     * 
     * 
     */
    private ChassisSpeeds makeSpeeds(
            ChassisSpeeds prev,
            double dx,
            double dy,
            double dtheta,
            double min_s) {
        if (DEBUG) {
            Util.printf("dx %8.5f dy %8.5f dtheta %8.5f s %8.5f\n", dx, dy, dtheta, min_s);
        }
        // double vx = prev.vxMetersPerSecond + min_s * dx;
        // double vy = prev.vyMetersPerSecond + min_s * dy;
        // double omega = prev.omegaRadiansPerSecond + min_s * dtheta;
        // return new ChassisSpeeds(vx, vy, omega);

        double omega = prev.omegaRadiansPerSecond + min_s * dtheta;
        double drift = -1.0 * omega * TimedRobot100.LOOP_PERIOD_S;
        double vx = prev.vxMetersPerSecond * Math.cos(drift)
                - prev.vyMetersPerSecond * Math.sin(drift)
                + min_s * dx;
        double vy = prev.vxMetersPerSecond * Math.sin(drift)
                + prev.vyMetersPerSecond * Math.cos(drift)
                + min_s * dy;
        return new ChassisSpeeds(vx, vy, omega);

    }
}
