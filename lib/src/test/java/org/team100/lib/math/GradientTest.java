package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Function;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class GradientTest {
    /** Scalar function, f(x) = x. */
    @Test
    void test1() {
        Nat<N1> cols = Nat.N1();
        Function<Vector<N1>, Double> f = x -> x.get(0);
        Vector<N1> x = VecBuilder.fill(0);
        Vector<N1> grad = NumericalGradient.numericalGradient(cols, f, x);
        assertEquals(1, grad.get(0), 1e-9);
    }

    /** Scalar function, f(x) = x^2. */
    @Test
    void test2() {
        Nat<N1> cols = Nat.N1();
        Function<Vector<N1>, Double> f = x -> Math.pow(x.get(0), 2);
        Vector<N1> x = VecBuilder.fill(1);
        Vector<N1> grad = NumericalGradient.numericalGradient(cols, f, x);
        assertEquals(2, grad.get(0), 1e-9);
    }

    /** Multivariate scalar function, f(x) = norm(x)^2 */
    @Test
    void test3() {
        Nat<N2> cols = Nat.N2();
        Function<Vector<N2>, Double> f = x -> Math.pow(x.normF(), 2);
        Vector<N2> x = VecBuilder.fill(1, 0.5);
        Vector<N2> grad = NumericalGradient.numericalGradient(cols, f, x);
        assertEquals(2, grad.get(0), 1e-9);
        assertEquals(1, grad.get(1), 1e-9);
    }
}
