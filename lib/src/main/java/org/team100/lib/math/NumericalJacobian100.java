package org.team100.lib.math;

import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * Similar to the WPI version but using vectors instead of matrices.
 * 
 * Estimates the Jacobian using symmetric differences around the reference x.
 */
public class NumericalJacobian100 {
    private static final double kEpsilon = 1e-5;

    /**
     * Estimates the Jacobian using symmetric differences around the reference x.
     */
    public static <Y extends Num, X extends Num> Matrix<Y, X> numericalJacobian(
            Nat<X> xdim,
            Nat<Y> ydim,
            Function<Vector<X>, Vector<Y>> f,
            Vector<X> x) {
        Matrix<Y, X> result = new Matrix<>(ydim, xdim);
        for (int i = 0; i < xdim.getNum(); i++) {
            Vector<X> dxPlus = new Vector<>(x.getStorage().copy());
            Vector<X> dxMinus = new Vector<>(x.getStorage().copy());
            dxPlus.set(i, 0, dxPlus.get(i, 0) + kEpsilon);
            dxMinus.set(i, 0, dxMinus.get(i, 0) - kEpsilon);
            Vector<Y> dF = f.apply(dxPlus).minus(f.apply(dxMinus)).div(2 * kEpsilon);
            result.setColumn(i, Matrix.changeBoundsUnchecked(dF));
        }
        return result;
    }

    /**
     * Estimates the Jacobian using a single-sided difference to the right of the
     * reference x.
     * 
     * Mutates x to save allocation cost, but puts it back the way it was upon
     * returning.
     */
    public static <Y extends Num, X extends Num> Matrix<Y, X> numericalJacobian2(
            Nat<X> xdim,
            Nat<Y> ydim,
            Function<Vector<X>, Vector<Y>> f,
            Vector<X> x) {
        Vector<Y> Y = f.apply(x);
        Matrix<Y, X> result = new Matrix<>(ydim, xdim);
        for (int i = 0; i < xdim.getNum(); i++) {
            double xi = x.get(i);
            x.set(i, 0, xi + kEpsilon);
            Vector<Y> Y1 = f.apply(x);
            // System.out.printf("Y1 - Y %s\n", Y1.minus(Y));
            // mutate Y1 here to save lots of allocations
            // dF = (f(x + epsilon) - f(x)) / epsilon
            for (int j = 0; j < ydim.getNum(); ++j) {
                Y1.set(j, 0, (Y1.get(j) - Y.get(j)) / kEpsilon);
            }
            result.setColumn(i, Y1);
            // put the old value back
            x.set(i, 0, xi);
        }
        return result;
    }
}
