package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Function;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalJacobian;

public class NumericalJacobianTest {
    /** Scalar function, f(x) = x. */
    @Test
    void test1() {
        Nat<N1> rows = Nat.N1();
        Nat<N1> cols = Nat.N1();
        Function<Matrix<N1, N1>, Matrix<N1, N1>> f = x -> x;
        Matrix<N1, N1> x = MatBuilder.fill(Nat.N1(), Nat.N1(), 0);
        Matrix<N1, N1> j = NumericalJacobian.numericalJacobian(rows, cols, f, x);
        assertEquals(1, j.get(0, 0), 1e-9);
    }

    /** Scalar function, f(x) = x^2. */
    @Test
    void test2() {
        Nat<N1> rows = Nat.N1();
        Nat<N1> cols = Nat.N1();
        Function<Matrix<N1, N1>, Matrix<N1, N1>> f = x -> x.pow(2);
        Matrix<N1, N1> x = MatBuilder.fill(Nat.N1(), Nat.N1(), 1);
        Matrix<N1, N1> j = NumericalJacobian.numericalJacobian(rows, cols, f, x);
        assertEquals(2, j.get(0, 0), 1e-9);
    }

    /** Multivariate scalar function, f(x) = norm(x)^2 */
    @Test
    void test3() {
        Nat<N1> rows = Nat.N1();
        Nat<N2> cols = Nat.N2();
        Function<Matrix<N2, N1>, Matrix<N1, N1>> f = x -> MatBuilder.fill(Nat.N1(), Nat.N1(), Math.pow(x.normF(), 2));
        Matrix<N2, N1> x = MatBuilder.fill(Nat.N2(), Nat.N1(), 1, 0.5);
        Matrix<N1, N2> j = NumericalJacobian.numericalJacobian(rows, cols, f, x);
        assertEquals(2, j.get(0, 0), 1e-9);
        assertEquals(1, j.get(0, 1), 1e-9);
    }

}
