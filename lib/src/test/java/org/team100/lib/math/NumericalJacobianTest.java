package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Function;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class NumericalJacobianTest {
    /** Scalar function, f(x) = x. */
    @Test
    void test1() {
        Nat<N1> xdim = Nat.N1();
        Nat<N1> ydim = Nat.N1();
        Function<Vector<N1>, Vector<N1>> f = x -> x;
        Vector<N1> x = VecBuilder.fill(0);
        Matrix<N1, N1> j = NumericalJacobian100.numericalJacobian(xdim, ydim, f, x);
        assertEquals(1, j.get(0, 0), 1e-9);
    }

    /** Scalar function, f(x) = 2x. */
    @Test
    void test2() {
        Nat<N1> xdim = Nat.N1();
        Nat<N1> ydim = Nat.N1();
        Function<Vector<N1>, Vector<N1>> f = x -> x.times(2);
        Vector<N1> x = VecBuilder.fill(1);
        Matrix<N1, N1> j = NumericalJacobian100.numericalJacobian(xdim, ydim, f, x);
        assertEquals(2, j.get(0, 0), 1e-9);
    }

    /** Multivariate scalar function, f(x) = norm(x)^2 */
    @Test
    void test3() {
        Nat<N2> xdim = Nat.N2();
        Nat<N1> ydim = Nat.N1();
        Function<Vector<N2>, Vector<N1>> f = x -> VecBuilder.fill(Math.pow(x.normF(), 2));
        Vector<N2> x = VecBuilder.fill(1, 0.5);
        Matrix<N1, N2> j = NumericalJacobian100.numericalJacobian(xdim, ydim, f, x);
        assertEquals(2, j.get(0, 0), 1e-9);
        assertEquals(1, j.get(0, 1), 1e-9);
    }

    /** Multivariate vector function, f(x) = x */
    @Test
    void test4() {
        Nat<N2> xdim = Nat.N2();
        Nat<N2> ydim = Nat.N2();
        Function<Vector<N2>, Vector<N2>> f = x -> x;
        Vector<N2> x = VecBuilder.fill(1, 1);
        Matrix<N2, N2> j = NumericalJacobian100.numericalJacobian(xdim, ydim, f, x);
        assertEquals(1, j.get(0, 0), 1e-9);
        assertEquals(0, j.get(0, 1), 1e-9);
        assertEquals(0, j.get(1, 0), 1e-9);
        assertEquals(1, j.get(1, 1), 1e-9);
        Matrix<N2, N2> jInv = j.inv();
        assertEquals(1, jInv.get(0, 0), 1e-9);
        assertEquals(0, jInv.get(0, 1), 1e-9);
        assertEquals(0, jInv.get(1, 0), 1e-9);
        assertEquals(1, jInv.get(1, 1), 1e-9);
    }

}
