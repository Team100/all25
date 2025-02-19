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

    public double maxSpeed() {
        double desired = 0;
        desired = Math.max(desired, Math.abs(frontLeft.speedMetersPerSecond()));
        desired = Math.max(desired, Math.abs(frontRight.speedMetersPerSecond()));
        desired = Math.max(desired, Math.abs(rearLeft.speedMetersPerSecond()));
        desired = Math.max(desired, Math.abs(rearRight.speedMetersPerSecond()));
        return desired;
    }

    /**
     * Make a copy with the supplied angle, where ours is empty.
     */
    public SwerveModuleStates overwriteEmpty(SwerveModuleStates other) {
        return new SwerveModuleStates(
                frontLeft.overwriteEmpty(other.frontLeft),
                frontRight.overwriteEmpty(other.frontRight),
                rearLeft.overwriteEmpty(other.rearLeft),
                rearRight.overwriteEmpty(other.rearRight));
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

    /**
     * Make a scaled copy.
     */
    public SwerveModuleStates scale(double scale) {
        return new SwerveModuleStates(
                frontLeft.scale(scale),
                frontRight.scale(scale),
                rearLeft.scale(scale),
                rearRight.scale(scale));
    }

    public SwerveModuleStates flipIfRequired(SwerveModuleStates prev) {
        return new SwerveModuleStates(
                frontLeft.flipIfRequired(prev.frontLeft),
                frontRight.flipIfRequired(prev.frontRight),
                rearLeft.flipIfRequired(prev.rearLeft),
                rearRight.flipIfRequired(prev.rearRight));
    }

    public double limit( SwerveModuleStates next, double limit) {
        double s = 1.0;
        s = Math.min(s, frontLeft().limit(next.frontLeft(), limit));
        s = Math.min(s, frontRight().limit(next.frontRight(), limit));
        s = Math.min(s, rearLeft().limit(next.rearLeft(), limit));
        s = Math.min(s, rearRight().limit(next.rearRight(), limit));
        return Math.max(s, 0);
    }
}
