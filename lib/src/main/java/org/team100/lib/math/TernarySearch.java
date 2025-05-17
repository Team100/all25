package org.team100.lib.math;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.util.Util;

/**
 * Ternary search finds the minimum by testing points. If the function has more
 * than one local minimum, there's no guarantee it will find the global minimum.
 * 
 * It's intended to find the minimum error in each step of coordinate descent,
 * so the function has a scalar input (i.e. configuration of one joint) and a
 * scalar output (i.e. distance).
 * 
 * This partitions equally and evaluates at each point, so it is not fast.
 */
public class TernarySearch {
    private final DoubleUnaryOperator m_f;
    private final double m_tolerance;
    private final int m_iterations;

    public TernarySearch(DoubleUnaryOperator f, double tolerance, int iterations) {
        m_f = f;
        m_tolerance = tolerance;
        m_iterations = iterations;
    }

    /** Find the minimum of f within [a,b]. */
    public double solve(double bottom, double top) {
        if (bottom > top)
            throw new IllegalArgumentException("bottom must be less than top");
        int i = 0;
        while (top - bottom > m_tolerance) {
            double d = (top - bottom) / 3;
            double low = bottom + d;
            double high = top - d;
            double flow = m_f.applyAsDouble(low);
            double fhigh = m_f.applyAsDouble(high);
            if (fhigh > flow)
                top = high;
            else
                bottom = low;
            i++;
            if (i > m_iterations) {
                Util.warn("iteration limit exceeded");
                return bottom;
            }
        }
        return bottom;
    }

}