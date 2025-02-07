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

    /**
     * For empty angles, use the supplied angle instead (e.g. the previous state).
     */
    public void overwriteEmpty(SwerveModuleStates prevModuleStates) {
        if (frontLeft.angle().isEmpty())
            frontLeft.angle = prevModuleStates.frontLeft().angle();
        if (frontRight.angle().isEmpty())
            frontRight.angle = prevModuleStates.frontRight().angle();
        if (rearLeft.angle().isEmpty())
            rearLeft.angle = prevModuleStates.rearLeft().angle();
        if (rearRight.angle().isEmpty())
            rearRight.angle = prevModuleStates.rearRight().angle();
    }
}
