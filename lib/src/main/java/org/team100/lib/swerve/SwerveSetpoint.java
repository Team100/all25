package org.team100.lib.swerve;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Used to track setpoints over time and enforce limits.
 */
public class SwerveSetpoint {
    /** Instantaneous speeds corresponding to the states. */
    private final ChassisSpeeds m_speeds;
    /** These should be actual, discretized states. */
    private final SwerveModuleStates m_states;

    /** New setpoint with zero speed and indeterminate steering */
    public SwerveSetpoint() {
        this(
                new ChassisSpeeds(),
                new SwerveModuleStates(
                        new SwerveModuleState100(0, Optional.empty()),
                        new SwerveModuleState100(0, Optional.empty()),
                        new SwerveModuleState100(0, Optional.empty()),
                        new SwerveModuleState100(0, Optional.empty())));
    }

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleStates initialStates) {
        m_speeds = chassisSpeeds;
        m_states = initialStates;
    }

    public ChassisSpeeds speeds() {
        return m_speeds;
    }

    public SwerveModuleStates states() {
        return m_states;
    }

    @Override
    public String toString() {
        return m_speeds.toString() + "  "
                + m_states.toString() + "\n";
    }
}
