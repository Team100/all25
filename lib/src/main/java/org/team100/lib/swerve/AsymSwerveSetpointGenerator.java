package org.team100.lib.swerve;

import java.util.function.DoubleSupplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AsymSwerveSetpointGenerator implements Glassy {
    private final SwerveKinodynamics m_limits;

    private final DriveAccelerationLimiter m_DriveAccelerationLimiter;

    public AsymSwerveSetpointGenerator(
            LoggerFactory parent,
            SwerveKinodynamics limits) {
        m_limits = limits;
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

        double min_s = 1.0;

        SwerveSetpoint target = new SwerveSetpoint(
                desiredSpeed,
                m_limits.toSwerveModuleStates(desiredSpeed));

        final SwerveModuleStates prevModuleStates = prevSetpoint.states();
        // the desired module state speeds are always positive.

        if (GeometryUtil.isZero(target.speeds())) {
            // use the previous module states if the desired speed is zero
            target = new SwerveSetpoint(target.speeds(), prevModuleStates.motionless());
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prev_vx = computeVx(prevModuleStates);
        double[] prev_vy = computeVy(prevModuleStates);

        double[] desired_vx = computeVx(target.states());
        double[] desired_vy = computeVy(target.states());

        double accel_min_s = m_DriveAccelerationLimiter.enforceWheelAccelLimit(
                prev_vx,
                prev_vy,
                desired_vx,
                desired_vy);
        min_s = Math.min(min_s, accel_min_s);

        return makeSetpoint(
                prevSetpoint,
                target,
                min_s);
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

    /** States are discretized. */
    private SwerveSetpoint makeSetpoint(
            final SwerveSetpoint prevSetpoint,
            SwerveSetpoint target,
            double min_s) {
        final SwerveModuleStates prevModuleStates = prevSetpoint.states();
        ChassisSpeeds setpointSpeeds = makeSpeeds(
                prevSetpoint.speeds(),
                target.speeds(),
                min_s);

        SwerveModuleStates setpointStates = m_limits.toSwerveModuleStates(setpointSpeeds);
        setpointStates = setpointStates.overwriteEmpty(prevModuleStates);
        setpointStates = setpointStates.flipIfRequired(prevModuleStates);

        // sync up the speeds with the states. is this needed?
        ChassisSpeeds finalSpeeds = m_limits.toChassisSpeedsWithDiscretization(
                setpointStates,
                TimedRobot100.LOOP_PERIOD_S);

        return new SwerveSetpoint(finalSpeeds, setpointStates);
    }

    /**
     * Scales the commanded accelerations by min_s, i.e. applies the constraints
     * calculated earlier. Chooses a course to match the target course, if
     * it's not too far from the initial course.
     */
    ChassisSpeeds makeSpeeds(ChassisSpeeds prev, ChassisSpeeds target, double min_s) {

        final ChassisSpeeds deltaV = target.minus(prev);

        ChassisSpeeds delta = deltaV.times(min_s);

        double deltaNorm = GeometryUtil.norm(delta);

        ChassisSpeeds lerp = prev.plus(delta);

        if (GeometryUtil.norm(target) < 1e-6) {
            return lerp;
        }
        ChassisSpeeds projectedLerp = GeometryUtil.project(lerp, target);
        ChassisSpeeds projectedDelta = projectedLerp.minus(prev);
        projectedLerp = prev.plus(projectedDelta);

        double projectedDeltaNorm = GeometryUtil.norm(projectedDelta);

        if (projectedDeltaNorm > deltaNorm) {
            return lerp;
        }

        return projectedLerp;
    }
}
