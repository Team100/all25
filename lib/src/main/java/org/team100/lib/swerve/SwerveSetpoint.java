package org.team100.lib.swerve;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Used to track setpoints over time and enforce limits.
 */
public class SwerveSetpoint {
    // is this the "instantaneous" chassis speed or the "discretized" one?
    // it seems bad to use the same type for both.
    // the "discretized" one is really a "twist" isn't it?
    private final ChassisSpeeds m_ChassisSpeeds;
    private final SwerveModuleStates m_ModuleStates;

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
        m_ChassisSpeeds = chassisSpeeds;
        m_ModuleStates = initialStates;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_ChassisSpeeds;
    }

    public SwerveModuleStates getModuleStates() {
        return m_ModuleStates;
    }

    @Override
    public String toString() {
        return m_ChassisSpeeds.toString() + "  "
                + m_ModuleStates.toString() + "\n";
    }
}
