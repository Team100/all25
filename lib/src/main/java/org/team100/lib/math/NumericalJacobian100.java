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
}
