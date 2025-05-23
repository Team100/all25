package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;
import java.util.function.Function;
import java.util.function.ToDoubleFunction;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

import edu.wpi.first.math.optimization.SimulatedAnnealing;

/**
 * Use the WPI SimulatedAnnealing class to do line search.
 * 
 * This is not actually a good idea at all for such a small and smooth search
 * space.
 * 
 * See TernarySearchTest.java.
 */
public class SimulatedAnnealingTest {

    @Test
    void testUnimodal() {
        Random random = new Random();
        ToDoubleFunction<Double> f = (x) -> Math.pow((x - 1), 2) + 1;
        Function<Double, Double> n = (x) -> x + random.nextGaussian(0, 0.1);
        SimulatedAnnealing<Double> s = new SimulatedAnnealing<>(1.0, n, f);
        assertEquals(1.0, s.solve(0.0, 1000), 1e-3);
    }

    /** 65 us per solve, 650X worse than ternary search. */
    @Test
    void testPerformance() {
        Random random = new Random();
        ToDoubleFunction<Double> f = (x) -> Math.pow((x - 1), 2) + 1;
        Function<Double, Double> n = (x) -> x + random.nextGaussian(0, 0.1);
        SimulatedAnnealing<Double> s = new SimulatedAnnealing<>(1.0, n, f);

        int iterations = 10000;
        long startTime = System.currentTimeMillis();

        for (int i = 0; i < iterations; ++i) {
            s.solve(0.0, 1000);
        }
        long finishTime = System.currentTimeMillis();

        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
    }
}
