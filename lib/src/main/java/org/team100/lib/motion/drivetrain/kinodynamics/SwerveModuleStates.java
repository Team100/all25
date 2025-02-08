package org.team100.lib.motion.drivetrain.kinodynamics;

/**
 * Container for swerve module states.
 * 
 * This is intended to avoid passing around an array of states,
 * and having to remember which location corresponds to which index.
 */
public record SwerveModuleStates(
        SwerveModuleState100 frontLeft,
        SwerveModuleState100 frontRight,
        SwerveModuleState100 rearLeft,
        SwerveModuleState100 rearRight) {
    /** For when you don't care about which is which. */
    public SwerveModuleState100[] all() {
        return new SwerveModuleState100[] {
                frontLeft,
                frontRight,
                rearLeft,
                rearRight
        };
    }

    SwerveModuleState100 overwrite (SwerveModuleState100 base, SwerveModuleState100 overwrite) {
        
    }
    /**
     * For empty angles, use the supplied angle instead (e.g. the previous state).
     */
    public void overwriteEmpty(SwerveModuleStates prevModuleStates) {
        SwerveModuleState100 fl;
        if (frontLeft.angle().isEmpty())
            fl = new SwerveModuleState100(
                    frontLeft.speedMetersPerSecond(),
                    prevModuleStates.frontLeft().angle());
        else
            fl = frontLeft;
    
        if (frontRight.angle().isEmpty())
            frontRight = new SwerveModuleState100(
                    frontRight.speedMetersPerSecond(),
                    prevModuleStates.frontRight().angle());
        if (rearLeft.angle().isEmpty())
            rearLeft = new SwerveModuleState100(
                    rearLeft.speedMetersPerSecond(),
                    prevModuleStates.rearLeft().angle());
        if (rearRight.angle().isEmpty())
            rearRight = new SwerveModuleState100(
                    rearRight.speedMetersPerSecond(),
                    prevModuleStates.rearRight().angle());
    }
}
