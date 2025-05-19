package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Function;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class CoordinateDescentTest {
    @Test
    void test1() {
        Nat<N1> rows = Nat.N1();
        Function<Vector<N1>, Double> f = x -> Math.pow(x.get(0), 2);
        CoordinateDescent<N1> s = new CoordinateDescent<>(rows, f, 1e-3, 100);
        Vector<N1> bottom = VecBuilder.fill(-4);
        Vector<N1> x = VecBuilder.fill(1);
        Vector<N1> top = VecBuilder.fill(4);
        Vector<N1> soln = s.solve(bottom, x, top);
        assertEquals(0, soln.get(0), 1e-3);
    }

    @Test
    void test2() {
        Nat<N2> rows = Nat.N2();
        Function<Vector<N2>, Double> f = x -> Math.pow(x.normF(), 2);
        CoordinateDescent<N2> s = new CoordinateDescent<>(rows, f, 1e-3, 100);
        Vector<N2> bottom = VecBuilder.fill(-4, -4);
        Vector<N2> x = VecBuilder.fill(1, 1);
        Vector<N2> top = VecBuilder.fill(4, 4);
        Vector<N2> soln = s.solve(bottom, x, top);
        assertEquals(0, soln.get(0), 1e-3);
        assertEquals(0, soln.get(1), 1e-3);
    }

    /** 0.7 us */
    @Test
    void testPerformance1() {
        Function<Vector<N1>, Double> f = (x) -> Math.pow((x.get(0) - 1), 2) + 1;
        CoordinateDescent<N1> s = new CoordinateDescent<>(Nat.N1(), f, 1e-3, 100);
        int iterations = 100000;
        long startTime = System.currentTimeMillis();
        Vector<N1> bottom = VecBuilder.fill(-4);
        Vector<N1> x = VecBuilder.fill(1);
        Vector<N1> top = VecBuilder.fill(4);
        for (int i = 0; i < iterations; ++i) {
            s.solve(bottom, x, top);
        }
        long finishTime = System.currentTimeMillis();
        Util.println("Coordinate descent over quadratic");
        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
    }

    /** 136 us per solve */
    @Test
    void testPerformance2() {
        Function<Vector<N2>, Double> f = (x) -> Math.pow((x.normF() - 1), 2) + 1;
        CoordinateDescent<N2> s = new CoordinateDescent<>(Nat.N2(), f, 1e-3, 100);
        // int iterations = 5000;
        int iterations = 1;
        long startTime = System.currentTimeMillis();
        Vector<N2> bottom = VecBuilder.fill(-4, -4);
        Vector<N2> x = VecBuilder.fill(1, 1);
        Vector<N2> top = VecBuilder.fill(4, 4);
        for (int i = 0; i < iterations; ++i) {
            s.solve(bottom, x, top);
        }
        long finishTime = System.currentTimeMillis();
        Util.println("Coordinate descent over quadratic");
        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
    }

}
