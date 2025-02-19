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
     * Make a copy with no drive component.
     */
    public SwerveModuleStates motionless() {
        return new SwerveModuleStates(
                new SwerveModuleState100(0, frontLeft.angle()),
                new SwerveModuleState100(0, frontRight.angle()),
                new SwerveModuleState100(0, rearLeft.angle()),
                new SwerveModuleState100(0, rearRight.angle()));
    }
}
