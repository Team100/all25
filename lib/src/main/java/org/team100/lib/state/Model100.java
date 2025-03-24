package org.team100.lib.state;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;

/**
 * One-dimensional system state, used for system modeling. The model only
 * contains position and velocity, there's no measurement of acceleration.
 * 
 * The usual state-space representation would be X = (x,v) and Xdot = (v,a).
 * Units are meters, radians, and seconds.
 */
public class Model100 {
    private final double m_x;
    private final double m_v;

    /** Specify position and velocity. */
    public Model100(double x, double v) {
        m_x = x;
        m_v = v;
    }

    public static Model100 x(double x) {
        return new Model100(x, 0);
    }

    public Model100() {
        this(0, 0);
    }

    public double x() {
        return m_x;
    }

    public double v() {
        return m_v;
    }

    /**
     * @return the control corresponding to this measurement, with zero
     *         acceleration.
     */
    public Control100 control() {
        return new Control100(m_x, m_v, 0);
    }

    public Model100 minus(Model100 other) {
        return new Model100(x() - other.x(), v() - other.v());
    }

    public Model100 plus(Model100 other) {
        return new Model100(x() + other.x(), v() + other.v());
    }

    public Model100 mult(double scaler) {
        return new Model100(m_x * scaler, m_v * scaler);
    }

    /** position and velocity are both within (the same) tolerance */
    public boolean near(Model100 other, double tolerance) {
        return MathUtil.isNear(m_x, other.m_x, tolerance) &&
                MathUtil.isNear(m_v, other.m_v, tolerance);
    }

    /** position is within xtolerance, velocity is within vtolerance. */
    public boolean near(Model100 other, double xTolerance, double vTolerance) {
        return MathUtil.isNear(m_x, other.m_x, xTolerance) &&
                MathUtil.isNear(m_v, other.m_v, vTolerance);
    }

    @Override
    public String toString() {
        return String.format("Model100(X %11.8f V %11.8f)", m_x, m_v);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof Model100) {
            Model100 rhs = (Model100) other;
            return this.m_x == rhs.m_x && this.m_v == rhs.m_v;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_x, m_v);
    }

}
