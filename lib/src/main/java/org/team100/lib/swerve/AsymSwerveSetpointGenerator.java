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
    private static final boolean DEBUG = false;
    // turns greater than this will flip
    // this used to be pi/2, which resulted in "square corner" paths
    private static final double flipLimitRad = 3 * Math.PI / 4;

    private final SwerveKinodynamics m_limits;

    private final SteeringOverride m_SteeringOverride;
    private final SteeringRateLimiter m_steeringRateLimiter;
    private final DriveAccelerationLimiter m_DriveAccelerationLimiter;

    public AsymSwerveSetpointGenerator(
            LoggerFactory parent,
            SwerveKinodynamics limits,
            DoubleSupplier batteryVoltage) {
        m_limits = limits;
        m_SteeringOverride = new SteeringOverride(parent, limits);
        m_steeringRateLimiter = new SteeringRateLimiter(parent, limits);
        m_DriveAccelerationLimiter = new DriveAccelerationLimiter(parent, limits);
    }

    /**
     * Generate a new setpoint.
     * 
     * Tries to find a new setpoint with the same course as the desired one, but
     * gives up if the desired course is too far from the previous one.
     */
    public SwerveSetpoint generateSetpoint(
            final SwerveSetpoint prevSetpoint,
            final ChassisSpeeds desiredSpeed) {
        if (DEBUG)
            Util.printf(
                    "generateSetpoint() prev:[vx %.8f vy %.8f omega %.8f course %.8f] desired:[vx %.8f vy %.8f omega %.8f course %.8f]\n",
                    prevSetpoint.speeds().vxMetersPerSecond,
                    prevSetpoint.speeds().vyMetersPerSecond,
                    prevSetpoint.speeds().omegaRadiansPerSecond,
                    GeometryUtil.getCourse(prevSetpoint.speeds()).orElse(new Rotation2d()).getRadians(),
                    desiredSpeed.vxMetersPerSecond,
                    desiredSpeed.vyMetersPerSecond,
                    desiredSpeed.omegaRadiansPerSecond,
                    GeometryUtil.getCourse(desiredSpeed).orElse(new Rotation2d()).getRadians());
        double min_s = 1.0;

        SwerveSetpoint target = desaturate(desiredSpeed);

        final SwerveModuleStates prevModuleStates = prevSetpoint.states();
        // the desired module state speeds are always positive.

        if (GeometryUtil.isZero(target.speeds())) {
            // use the previous module states if the desired speed is zero
            target = new SwerveSetpoint(target.speeds(), prevModuleStates.motionless());
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prev_vx = computeVx(prevModuleStates);
        double[] prev_vy = computeVy(prevModuleStates);
        // elements may be null.
        Rotation2d[] prev_heading = computeHeading(prevModuleStates);

        double[] desired_vx = computeVx(target.states());
        double[] desired_vy = computeVy(target.states());
        // elements may be null.
        Rotation2d[] desired_heading = computeHeading(target.states());

        boolean shouldStopAndReverse = shouldStopAndReverse(prev_heading, desired_heading);
        if (shouldStopAndReverse
                && !GeometryUtil.isZero(prevSetpoint.speeds())
                && !GeometryUtil.isZero(target.speeds())) {
            // It will (likely) be faster to stop the robot, rotate the modules in place to
            // the complement of the desired angle, and accelerate again.
            return generateSetpoint(prevSetpoint, new ChassisSpeeds());
        }

        // In cases where an individual module is stopped, we want to remember the right
        // steering angle to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically
        // lazy).
        SwerveModuleState100[] prevModuleStatesAll = prevModuleStates.all();
        Rotation2d[] overrideSteering = new Rotation2d[prevModuleStatesAll.length];

        if (GeometryUtil.isZero(target.speeds())) {
            for (int i = 0; i < prevModuleStatesAll.length; ++i) {
                if (prevModuleStatesAll[i].angle().isEmpty()) {
                    overrideSteering[i] = null;
                } else {
                    overrideSteering[i] = prevModuleStatesAll[i].angle().get();
                }
            }
        } else {
            double override_min_s = m_SteeringOverride.overrideIfStopped(
                    target.states(),
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

        return makeSetpoint(
                prevSetpoint,
                target,
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

    /**
     * Produce a setpoint that matches the desired course but respects velocity
     * limits.
     */
    SwerveSetpoint desaturate(ChassisSpeeds desiredState) {
        SwerveModuleStates desiredModuleStates = m_limits.toSwerveModuleStates(desiredState);
        if (m_limits.getMaxDriveVelocityM_S() > 0.0) {
            double desired = desiredModuleStates.maxSpeed();
            if (desired <= m_limits.getMaxDriveVelocityM_S()) {
                // all module states are feasible
                return new SwerveSetpoint(desiredState, desiredModuleStates);
            }
            double scale = m_limits.getMaxDriveVelocityM_S() / desired;

            SwerveModuleStates scaledStates = desiredModuleStates.scale(scale);
            ChassisSpeeds scaledSpeed = desiredState.times(scale);
            return new SwerveSetpoint(scaledSpeed, scaledStates);
        }
        return new SwerveSetpoint(desiredState, desiredModuleStates);
    }

    /** States are discretized. */
    private SwerveSetpoint makeSetpoint(
            final SwerveSetpoint prevSetpoint,
            SwerveSetpoint target,
            double min_s,
            Rotation2d[] overrideSteering) {
        final SwerveModuleStates prevModuleStates = prevSetpoint.states();
        if (DEBUG)
            Util.printf("makeSetpoint()\n");
        ChassisSpeeds setpointSpeeds = makeSpeeds(
                prevSetpoint.speeds(),
                target.speeds(),
                min_s);
        if (DEBUG)
            Util.printf("makeSetpoint() setpointSpeeds vx %.8f vy %.8f omega %.8f course %.8f\n",
                    setpointSpeeds.vxMetersPerSecond,
                    setpointSpeeds.vyMetersPerSecond,
                    setpointSpeeds.omegaRadiansPerSecond,
                    GeometryUtil.getCourse(setpointSpeeds).orElse(new Rotation2d()).getRadians());
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
                    prevSetpoint.speeds(), setpointSpeeds, finalSpeeds);
        return new SwerveSetpoint(finalSpeeds, setpointStates);
    }

    /**
     * Scales the commanded accelerations by min_s, i.e. applies the constraints
     * calculated earlier.  Chooses a course to match the target course, if
     * it's not too far from the initial course.
     */
    ChassisSpeeds makeSpeeds(ChassisSpeeds prev, ChassisSpeeds target, double min_s) {

        final ChassisSpeeds deltaV = target.minus(prev);

        // this does the right thing with the course if min_s is 1.0, otherwise not.
        if (DEBUG) {
            Util.printf(
                    "makeSpeeds() prev:[vx %.8f vy %.8f omega %.8f course %.8f]\n",
                    prev.vxMetersPerSecond,
                    prev.vyMetersPerSecond,
                    prev.omegaRadiansPerSecond,
                    GeometryUtil.getCourse(prev).orElse(new Rotation2d()).getRadians());
            Util.printf(
                    "makeSpeeds() target:[vx %.8f vy %.8f omega %.8f course %.8f]\n",
                    target.vxMetersPerSecond,
                    target.vyMetersPerSecond,
                    target.omegaRadiansPerSecond,
                    GeometryUtil.getCourse(target).orElse(new Rotation2d()).getRadians());
            Util.printf(
                    "makeSpeeds() deltaV:[dx %.8f dy %.8f dtheta %.8f] s %.8f\n",
                    deltaV.vxMetersPerSecond, deltaV.vyMetersPerSecond, deltaV.omegaRadiansPerSecond, min_s);
        }

        // interpolate between the prev and target
        ChassisSpeeds delta = deltaV.times(min_s);

        if (DEBUG)
            Util.printf("makeSpeeds() delta:[vx %.8f vy %.8f omega %.8f]\n",
                    delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond);
        double deltaNorm = GeometryUtil.norm(delta);
        if (DEBUG)
            Util.printf("makeSpeeds() delta norm %.8f\n",
                    deltaNorm);

        ChassisSpeeds lerp = prev.plus(delta);

        if (DEBUG)
            Util.printf("makeSpeeds() lerp:[vx %.8f vy %.8f omega %.8f course %.8f]\n",
                    lerp.vxMetersPerSecond,
                    lerp.vyMetersPerSecond,
                    lerp.omegaRadiansPerSecond,
                    GeometryUtil.getCourse(lerp).orElse(new Rotation2d()).getRadians());

        // adjust the interpolated speed so that it points in the target direction.
        if (GeometryUtil.norm(target) < 1e-6) {
            // the target is motionless, so there's no way to maintain the target course
            // the best we can do is maintain our current course
            return lerp;
        }
        ChassisSpeeds projectedLerp = GeometryUtil.project(lerp, target);
        ChassisSpeeds projectedDelta = projectedLerp.minus(prev);
        // projectedDelta = projectedDelta.times(0.9);
        projectedLerp = prev.plus(projectedDelta);
        if (DEBUG)
            Util.printf("makeSpeeds() projectedLerp:[vx %.8f vy %.8f omega %.8f course %.8f]\n",
                    projectedLerp.vxMetersPerSecond,
                    projectedLerp.vyMetersPerSecond,
                    projectedLerp.omegaRadiansPerSecond,
                    GeometryUtil.getCourse(projectedLerp).orElse(new Rotation2d()).getRadians());
        if (DEBUG)
            Util.printf("makeSpeeds() projectedDelta:[vx %.8f vy %.8f omega %.8f]\n",
                    projectedDelta.vxMetersPerSecond, projectedDelta.vyMetersPerSecond,
                    projectedDelta.omegaRadiansPerSecond);
        double projectedDeltaNorm = GeometryUtil.norm(projectedDelta);
        if (DEBUG)
            Util.printf("makeSpeeds() delta projected delta norm %.8f\n",
                    projectedDeltaNorm);
        if (projectedDeltaNorm > deltaNorm) {
            if (DEBUG)
                Util.printf("use lerp instead\n");
            // the projection is outside the interpolation envelope.
            return lerp;
        }

        return projectedLerp;
    }
}
