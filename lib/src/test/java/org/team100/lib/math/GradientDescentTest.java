package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

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

    /** Not particularly fast, 2 us per solve on my laptop. */
    @Test
    void testPerformance() {
        Function<Vector<N1>, Double> f = (x) -> Math.pow((x.get(0) - 1), 2) + 1;
        GradientDescent<N1> s = new GradientDescent<>(Nat.N1(), f, 1e-3, 100);
        int iterations = 1000000;
        long startTime = System.currentTimeMillis();
        Vector<N1> x = VecBuilder.fill(4);
        for (int i = 0; i < iterations; ++i) {
            s.solve(x);
        }
        long finishTime = System.currentTimeMillis();
        Util.println("Gradient descent over quadratic");
        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
    }

    /** 14 us per solve */
    @Test
    void testPerformance2() {
        Function<Vector<N2>, Double> f = (x) -> Math.pow((x.normF() - 1), 2) + 1;
        GradientDescent<N2> s = new GradientDescent<>(Nat.N2(), f, 1e-3, 100);
        // int iterations = 30000;
        int iterations = 1;
        long startTime = System.currentTimeMillis();
        Vector<N2> x = VecBuilder.fill(4, 4);
        for (int i = 0; i < iterations; ++i) {
            s.solve(x);
        }
        long finishTime = System.currentTimeMillis();
        Util.println("Gradient descent over quadratic");
        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
    }

    @Test
    void testPose() {
        Pose3d desired = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(0, 0, 1));
        Vector<N3> axis = VecBuilder.fill(0, 0, 1);
        Function<Vector<N1>, Double> f = (x) -> {
            Pose3d sample = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(axis, x.get(0)));
            Twist3d t = desired.log(sample);
            return GeometryUtil.norm(t);
        };
        GradientDescent<N1> s = new GradientDescent<>(Nat.N1(), f, 1e-12, 100);
        Vector<N1> x = VecBuilder.fill(0);
        assertEquals(1.0, s.solve(x).get(0), 1e-12);
    }

    /** 31 us per solve on my laptop, slower than ternary. */
    @Test
    void testPosePerformance() {
        Pose3d desired = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(0, 0, 1));
        Vector<N3> axis = VecBuilder.fill(0, 0, 1);
        Function<Vector<N1>, Double> f = (x) -> {
            Pose3d sample = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(axis, x.get(0)));
            Twist3d t = desired.log(sample);
            return GeometryUtil.norm(t);
        };
        GradientDescent<N1> s = new GradientDescent<>(Nat.N1(), f, 1e-3, 100);

        int iterations = 50000;
        Vector<N1> x = VecBuilder.fill(0);
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            s.solve(x);
        }
        long finishTime = System.currentTimeMillis();

        Util.println("Gradient descent over tangent norm");
        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
    }

}
