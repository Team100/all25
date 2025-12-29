package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.PathPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveUtil;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.tuning.Mutable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Linear velocity limit based on spatial yaw rate and drive wheel speed limit.
 * 
 * Slows the path velocity to accommodate the desired yaw rate.
 * 
 * This *should* provide the same answer as the YawRateConstraint, if the
 * omega limit calculation is correct.
 */
public class SwerveDriveDynamicsConstraint implements TimingConstraint {
    private static final boolean DEBUG = false;
    private final SwerveKinodynamics m_limits;
    private final Mutable vScale;
    private final Mutable aScale;

    /** Use the factory. */
    public SwerveDriveDynamicsConstraint(
            LoggerFactory parent,
            SwerveKinodynamics limits,
            double vScale,
            double aScale) {
        LoggerFactory log = parent.type(this);
        m_limits = limits;
        this.vScale = new Mutable(log, "vScale", vScale);
        this.aScale = new Mutable(log, "aScale", aScale);
    }

    /**
     * Given a target spatial heading rate (rad/m), return the maximum translational
     * speed allowed (m/s) that maintains the target spatial heading rate.
     */
    @Override
    public double maxV(PathPoint state) {
        // First check instantaneous velocity and compute a limit based on drive
        // velocity.
        Rotation2d course = state.waypoint().course().toRotation();
        Rotation2d heading = state.waypoint().pose().getRotation();
        Rotation2d course_local = course.minus(heading);
        double vx = course_local.getCos();
        double vy = course_local.getSin();
        // rad/m
        double vtheta = state.getHeadingRateRad_M();

        // first compute the effect of heading rate

        // this is a "spatial speed," direction and rad/m
        // which is like moving 1 m/s.
        ChassisSpeeds chassis_speeds = new ChassisSpeeds(vx, vy, vtheta);

        SwerveModuleStates module_states = m_limits.toSwerveModuleStates(chassis_speeds);
        double max_vel = Double.POSITIVE_INFINITY;
        for (SwerveModuleState100 module : module_states.all()) {
            max_vel = Math.min(max_vel, maxV() / Math.abs(module.speedMetersPerSecond()));
        }
        return max_vel;
    }

    double maxV() {
        return vScale.getAsDouble() * m_limits.getMaxDriveVelocityM_S();
    }

    /**
     * Provide current-limited and back-emf-limited acceleration limits.
     * 
     * Decel is unaffected by back EMF.
     * 
     * @see SwerveUtil.getAccelLimit()
     */
    @Override
    public double maxAccel(PathPoint state, double velocity) {
        if (Double.isNaN(velocity))
            throw new IllegalArgumentException();
        double maxAccel = SwerveUtil.minAccel(m_limits, 1, 1, velocity);
        // i think this only works because it's not *exactly* zero at full speed.
        if (Math.abs(maxAccel) < 1e-3)
            maxAccel = 0.0;
        if (DEBUG)
            System.out.printf("SWERVE CONSTRAINT %f\n", maxAccel);
        return maxAccel;
    }

    @Override
    public double maxDecel(PathPoint state, double velocity) {
        // min accel is stronger than max accel
        return -1.0 * maxA();
    }

    private double maxA() {
        return aScale.getAsDouble() * m_limits.getMaxDriveDecelerationM_S2();
    }
}