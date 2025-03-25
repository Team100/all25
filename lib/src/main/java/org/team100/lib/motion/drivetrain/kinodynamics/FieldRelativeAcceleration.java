package org.team100.lib.motion.drivetrain.kinodynamics;

import edu.wpi.first.math.MathUtil;

/** Meters, radians, and seconds. */
public record FieldRelativeAcceleration(double x, double y, double theta) {
    public FieldRelativeVelocity integrate(double dtSec) {
        return new FieldRelativeVelocity(x * dtSec, y * dtSec, theta * dtSec);
    }

    public FieldRelativeAcceleration plus(FieldRelativeAcceleration other) {
        return new FieldRelativeAcceleration(x + other.x, y + other.y, theta + other.theta);
    }

    public FieldRelativeAcceleration minus(FieldRelativeAcceleration other) {
        return new FieldRelativeAcceleration(x - other.x, y - other.y, theta - other.theta);
    }

    public FieldRelativeJerk jerk(FieldRelativeAcceleration previous, double dt) {
        FieldRelativeAcceleration v = minus(previous).div(dt);
        return new FieldRelativeJerk(v.x(), v.y(), v.theta());
    }

    public FieldRelativeAcceleration times(double scalar) {
        return new FieldRelativeAcceleration(x * scalar, y * scalar, theta * scalar);
    }

    public FieldRelativeAcceleration div(double scalar) {
        return new FieldRelativeAcceleration(x / scalar, y / scalar, theta / scalar);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public static FieldRelativeAcceleration diff(
            FieldRelativeVelocity v1,
            FieldRelativeVelocity v2,
            double dtSec) {
        FieldRelativeVelocity dv = v2.minus(v1);
        return new FieldRelativeAcceleration(dv.x() / dtSec, dv.y() / dtSec, dv.theta() / dtSec);
    }

    public FieldRelativeAcceleration clamp(double maxAccel, double maxAlpha) {
        double norm = Math.hypot(x, y);
        double ratio = 1.0;
        if (norm > 1e-3 && norm > maxAccel) {
            ratio = maxAccel / norm;
        }
        return new FieldRelativeAcceleration(ratio * x, ratio * y, MathUtil.clamp(theta, -maxAlpha, maxAlpha));
    }

    @Override
    public String toString() {
        return String.format("(%5.2f, %5.2f, %5.2f)", x, y, theta);
    }
}
