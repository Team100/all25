package org.team100.lib.math;

import java.util.function.Function;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * Gradient is scalar Jacobian. This is cribbed from WPI's
 * NumericalJacobian.java, just to make it simpler for folks who aren't familiar
 * with the Jacobian concept, matrices, etc.
 */
public class NumericalGradient {
    private static final double kEpsilon = 1e-5;

    /**
     * Computes the numerical gradient with respect to x for f(x), by looking in the
     * neighborhood of x and computing the partial derivatives using symmetric
     * differences.
     */
    public static <R extends Num> Vector<R> numericalGradient(
            Nat<R> cols,
            Function<Vector<R>, Double> f,
            Vector<R> x) {
        Vector<R> result = new Vector<>(cols);
        for (int i = 0; i < cols.getNum(); i++) {
            Vector<R> dxPlus = new Vector<>(x.getStorage().copy());
            Vector<R> dxMinus = new Vector<>(x.getStorage().copy());
            dxPlus.set(i, 0, dxPlus.get(i, 0) + kEpsilon);
            dxMinus.set(i, 0, dxMinus.get(i, 0) - kEpsilon);
            Double fPlus = f.apply(dxPlus);
            Double fMinus = f.apply(dxMinus);
            double dF = (fPlus - fMinus) / (2 * kEpsilon);
            result.set(i, 0, dF);
        }
        return result;
    }
}
