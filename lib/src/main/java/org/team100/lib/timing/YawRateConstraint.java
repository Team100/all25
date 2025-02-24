package org.team100.lib.timing;

import java.util.Optional;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Linear velocity limit based on spatial yaw rate, drivetrain omega limit
 * (scaled), and drivetrain alpha limit (scaled).
 * 
 * Slows the path velocity to accommodate the desired yaw rate.
 * 
 * Does not affect maximum acceleration.
 */
public class YawRateConstraint implements TimingConstraint {
    private final double m_maxOmegaRad_S;
    private final double m_maxAlphaRad_S2;

    /**
     * Use the factory.
     * 
     * @param limits absolute maxima
     * @param scale  apply to the maximum angular speed to get the actual
     *               constraint. The absolute maximum yaw rate is *very* high, and
     *               never useful for trajectories. A good number to try here might
     *               be 0.2.
     */
    public YawRateConstraint(SwerveKinodynamics limits, double scale) {
        m_maxOmegaRad_S = limits.getMaxAngleSpeedRad_S() * scale;
        m_maxAlphaRad_S2 = limits.getMaxAngleAccelRad_S2() * scale;
    }

    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        Optional<Rotation2d> course = state.getCourse();
        if (course.isEmpty()) {
            // This is turn in place.
            return new NonNegativeDouble(Double.MAX_VALUE);
        }
        // Heading rate in rad/m
        final double heading_rate = state.getHeadingRate();
        // rad/s / rad/m => m/s.
        return new NonNegativeDouble(m_maxOmegaRad_S / Math.abs(heading_rate));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        Optional<Rotation2d> course = state.getCourse();
        if (course.isEmpty()) {
            // This is turn in place, which doesn't really work anyway.
            return MinMaxAcceleration.kNoLimits;
        }
        // Heading rate in rad/m
        final double heading_rate = state.getHeadingRate();
        // rad/s^2 / rad/m => m/s^2
        double limitM_S = m_maxAlphaRad_S2 / Math.abs(heading_rate);
        return new MinMaxAcceleration(-limitM_S, limitM_S);
    }
}