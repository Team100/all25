package org.team100.lib.state;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * One-dimensional system state, used for system modeling. The model only
 * contains position and velocity, there's no measurement of acceleration.
 * 
 * The usual state-space representation would be X = (x,v) and Xdot = (v,a).
 * Units are meters, radians, and seconds.
 * 
 * @param x position
 * @param v velocity
 */
public record Model100(double x, double v) implements Interpolatable<Model100> {

    public Model100() {
        this(0, 0);
    }

    /**
     * @return the control corresponding to this measurement, with zero
     *         acceleration.
     */
    public Control100 control() {
        return new Control100(x, v, 0);
    }

    public Model100 minus(Model100 other) {
        return new Model100(x() - other.x(), v() - other.v());
    }

    public Model100 plus(Model100 other) {
        return new Model100(x() + other.x(), v() + other.v());
    }

    public Model100 mult(double scalar) {
        return new Model100(x * scalar, v * scalar);
    }

    /**
     * True if not null and position and velocity are both within (the same)
     * tolerance
     */
    public boolean near(Model100 other, double tolerance) {
        return other != null
                && MathUtil.isNear(x, other.x, tolerance)
                && MathUtil.isNear(v, other.v, tolerance);
    }

    /**
     * True if not null, position is within xtolerance, velocity is within
     * vtolerance.
     */
    public boolean near(Model100 other, double xTolerance, double vTolerance) {
        return other != null
                && MathUtil.isNear(x, other.x, xTolerance)
                && MathUtil.isNear(v, other.v, vTolerance);
    }

    @Override
    public Model100 interpolate(Model100 endValue, double t) {
        return new Model100(
                MathUtil.interpolate(x, endValue.x, t),
                MathUtil.interpolate(v, endValue.v, t));

    }

    @Override
    public String toString() {
        return String.format("Model100(X %11.8f V %11.8f)", x, v);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof Model100) {
            Model100 rhs = (Model100) other;
            return this.x == rhs.x && this.v == rhs.v;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, v);
    }
}
