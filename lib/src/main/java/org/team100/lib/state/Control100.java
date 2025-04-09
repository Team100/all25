package org.team100.lib.state;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;

/**
 * One-dimensional system state, used for control, so it includes acceleration,
 * which could be part of the control output.
 * 
 * The usual state-space representation would be X = (x,v) and Xdot = (v,a).
 * Units are meters, radians, and seconds.
 * 
 * @param x position
 * @param v velocity
 * @param a acceleration
 */
public record Control100(double x, double v, double a) {

    public Control100(double x, double v) {
        this(x, v, 0);
    }

    public Control100() {
        this(0, 0, 0);
    }

    /**
     * Return the model corresponding to this control, i.e. without acceleration.
     */
    public Model100 model() {
        return new Model100(x, v);
    }

    public Control100 minus(Control100 other) {
        return new Control100(x() - other.x(), v() - other.v(), a() - other.a());
    }

    public Control100 plus(Control100 other) {
        return new Control100(x() + other.x(), v() + other.v(), a() + other.a());
    }

    public Control100 mult(double scalar) {
        return new Control100(x * scalar, v * scalar, a * scalar);
    }

    public boolean near(Control100 other, double tolerance) {
        return MathUtil.isNear(x, other.x, tolerance) &&
                MathUtil.isNear(v, other.v, tolerance);
    }

    @Override
    public String toString() {
        return String.format("Control100(X %5.3f V %5.3f A %5.3f)", x, v, a);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof Control100) {
            Control100 rhs = (Control100) other;
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
