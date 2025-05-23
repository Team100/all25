package org.team100.lib.math;

import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * Newton's method finds a zero of a multivariate function; in this case, the
 * function is the error between the f and the desired value of f, so
 * driving it to zero yields the x values to get the desired f.
 * 
 * TODO: would it be better (i.e. faster) to make the error a scalar metric?
 * 
 * Uses the (estimated) Jacobian of the function to estimate the x intercept.
 * 
 * https://en.wikipedia.org/wiki/Newton%27s_method
 * https://hades.mech.northwestern.edu/images/7/7f/MR.pdf
 */
public class NewtonsMethod<X extends Num, Y extends Num> {
    private final Nat<X> m_xdim;
    private final Nat<Y> m_ydim;
    private final Function<Vector<X>, Vector<Y>> m_f;
    private final double m_tolerance;
    private final double m_toleranceSq;
    private final int m_iterations;

    public NewtonsMethod(
            Nat<X> xdim,
            Nat<Y> ydim,
            Function<Vector<X>, Vector<Y>> f,
            double tolerance,
            int iterations) {
        m_xdim = xdim;
        m_ydim = ydim;
        m_f = f;
        m_tolerance = tolerance;
        m_toleranceSq = tolerance * tolerance;
        m_iterations = iterations;
    }

    public Vector<X> solve(Vector<X> initial, Vector<Y> goal) {
        Vector<X> x = new Vector<>(initial.getStorage().copy());
        for (int i = 0; i < m_iterations; ++i) {
            Vector<Y> y = m_f.apply(x);
            Vector<Y> error = goal.minus(y);
            if (error.norm() < m_tolerance)
                return x;
            Matrix<Y, X> j = NumericalJacobian100.numericalJacobian(m_xdim, m_ydim, m_f, x);
            Vector<X> dx = new Vector<>(j.solve(error));
            x = x.plus(dx);
        }
        return x;
    }

    /** Use single-sided Jacobian */
    public Vector<X> solve2(Vector<X> initial, Vector<Y> goal) {
        Vector<X> x = new Vector<>(initial.getStorage().copy());
        for (int i = 0; i < m_iterations; ++i) {
            Vector<Y> y = m_f.apply(x);
            Vector<Y> error = goal.minus(y);
            // using dot instead of norm saves the sqrt.
            if (error.dot(error) < m_toleranceSq)
                return x;
            Matrix<Y, X> j = NumericalJacobian100.numericalJacobian2(m_xdim, m_ydim, m_f, x);
            System.out.printf("J %s\n", j);
            Vector<X> dx = new Vector<>(j.solve(error));
            // mutate x directly here to save allocations
            // x = x + dx
            for (int ji = 0; ji < x.getNumRows(); ++ji) {
                x.set(ji, 0, x.get(ji, 0) + dx.get(ji, 0));
            }
        }
        return x;
    }

}
