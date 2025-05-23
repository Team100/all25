package org.team100.lib.math;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/** Line search over each coordinate. */
public class CoordinateDescent<R extends Num> {
    private final Nat<R> m_rows;
    private final Function<Vector<R>, Double> m_f;
    private final double m_tolerance;
    private final int m_iterations;

    public CoordinateDescent(
            Nat<R> rows,
            Function<Vector<R>, Double> f,
            double tolerance,
            int iterations) {
        m_rows = rows;
        m_f = f;
        m_tolerance = tolerance;
        m_iterations = iterations;
    }

    public Vector<R> solve(final Vector<R> bottom, final Vector<R> initial, final Vector<R> top) {
        Vector<R> current = initial;
        for (int i = 0; i < m_iterations; ++i) {
            Vector<R> next = new Vector<>(current.getStorage().copy());
            // System.out.printf("%5.3f\n", next.get(0));
            for (int j = 0; j < m_rows.getNum(); ++j) {
                final int jj = j;
                DoubleUnaryOperator cf = (x) -> {
                    Vector<R> v = new Vector<>(next.getStorage().copy());
                    v.set(jj, 0, x);
                    return m_f.apply(v);
                };
                GoldenSectionSearch s = new GoldenSectionSearch(cf, m_tolerance, m_iterations);
                double x = s.solve(bottom.get(j), top.get(j));
                next.set(jj, 0, x);
            }
            double step = next.minus(current).norm();
            if (step < m_tolerance) {
                // System.out.println(i);
                return next;
            }
            current = next;
        }
        return current;
    }

}
