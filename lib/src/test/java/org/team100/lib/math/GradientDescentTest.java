package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Function;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public class GradientDescentTest {
    @Test
    void test1() {
        Function<Vector<N2>, Double> f = x -> Math.pow(x.normF(), 2);
        Vector<N2> x = VecBuilder.fill(1, 0.5);
        GradientDescent<N2> g = new GradientDescent<>(Nat.N2(), f, 1e-4, 100);
        Vector<N2> soln = g.solve(x);
        assertEquals(0, soln.get(0), 1e-3);
        assertEquals(0, soln.get(1), 1e-3);
           
    }
}
