package org.team100.lib.motion.drivetrain.kinodynamics;

public record FieldRelativeJerk(double x, double y, double theta) {
    public double norm() {
        return Math.hypot(x, y);
    }

    public FieldRelativeJerk times(double scalar) {
        return new FieldRelativeJerk(x * scalar, y * scalar, theta * scalar);
    }

    public FieldRelativeAcceleration integrate(double dtSec) {
        return new FieldRelativeAcceleration(x * dtSec, y * dtSec, theta * dtSec);
    }
}
