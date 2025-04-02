package org.team100.lib.timing;

import java.util.Optional;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveUtil;

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
    private final SwerveKinodynamics m_limits;
    private final double vScale;
    private final double aScale;

    /** Use the factory. */
    public SwerveDriveDynamicsConstraint(
            SwerveKinodynamics limits,
            double vScale,
            double aScale) {
        m_limits = limits;
        this.vScale = vScale;
        this.aScale = aScale;
    }

    /**
     * Given a target spatial heading rate (rad/m), return the maximum translational
     * speed allowed (m/s) that maintains the target spatial heading rate.
     */
    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        // First check instantaneous velocity and compute a limit based on drive
        // velocity.
        Optional<Rotation2d> course = state.getCourse();
        Rotation2d course_local = state.getHeading().unaryMinus()
                .rotateBy(course.isPresent() ? course.get() : Rotation2d.kZero);
        double vx = course_local.getCos();
        double vy = course_local.getSin();
        // rad/m
        double vtheta = state.getHeadingRate();

        // first compute the effect of heading rate

        // this is a "spatial speed," direction and rad/m
        // which is like moving 1 m/s.
        ChassisSpeeds chassis_speeds = new ChassisSpeeds(vx, vy, vtheta);

        SwerveModuleStates module_states = m_limits.toSwerveModuleStates(chassis_speeds);
        double max_vel = Double.POSITIVE_INFINITY;
        for (SwerveModuleState100 module : module_states.all()) {
            max_vel = Math.min(max_vel, maxV() / Math.abs(module.speedMetersPerSecond()));
        }
        return new NonNegativeDouble(max_vel);
    }

    private double maxV() {
        return vScale * m_limits.getMaxDriveVelocityM_S();
    }

    /**
     * Provide current-limited and back-emf-limited acceleration limits.
     * 
     * Decel is unaffected by back EMF.
     * 
     * @see SwerveUtil.getAccelLimit()
     */
    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        if (Double.isNaN(velocity))
            throw new IllegalArgumentException();
        // min accel is stronger than max accel
        double minAccel = -1.0 * maxA();
        double maxAccel = SwerveUtil.minAccel(m_limits, 1, 1, velocity);
        return new MinMaxAcceleration(
                minAccel,
                maxAccel);
    }

    private double maxA() {
        return aScale * m_limits.getMaxDriveDecelerationM_S2();
    }
}