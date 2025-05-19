package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleUnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.util.Util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N3;

public class GoldenSectionSearchTest {

    /** This function has only one minimum, so we find it. */
    @Test
    void testUnimodal() {
        DoubleUnaryOperator f = (x) -> Math.pow((x - 1), 2) + 1;
        GoldenSectionSearch s = new GoldenSectionSearch(f, 1e-6, 100);
        assertEquals(1.0, s.solve(-4, 4), 1e-6);
    }

    /**
     * This function has two minima that are almost the same. The ternary search
     * works in both cases because the lower minimum is surrounded by lower
     * territory, and the test points are initially far apart.
     */
    @Test
    void testBimodal() {
        {
            DoubleUnaryOperator f = (x) -> Math.pow(x, 4) - 3 * Math.pow(x, 2) + 0.1 * x + 1;
            GoldenSectionSearch s = new GoldenSectionSearch(f, 1e-6, 100);
            assertEquals(-1.232995, s.solve(-2, 2), 1e-6);
        }
        {
            // this swaps the upper and lower minimum
            DoubleUnaryOperator f = (x) -> Math.pow(x, 4) - 3 * Math.pow(x, 2) - 0.1 * x + 1;
            GoldenSectionSearch s = new GoldenSectionSearch(f, 1e-6, 100);
            assertEquals(1.232994, s.solve(-2, 2), 1e-6);
        }
    }

    /**  70 ns per solve on my laptop */
    @Test
    void testPerformance() {
        DoubleUnaryOperator f = (x) -> Math.pow((x - 1), 2) + 1;
        GoldenSectionSearch s = new GoldenSectionSearch(f, 1e-3, 100);
        int iterations = 500000;
        long startTime = System.currentTimeMillis();

        for (int i = 0; i < iterations; ++i) {
            s.solve(-4, 4);
        }
        long finishTime = System.currentTimeMillis();

        Util.println("Golden section search on quadratic");
        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
    }

    /**
     * This uses the natural pose metric, which is the norm of the tangent. It
     * involves a lot of computation.
     */
    @Test
    void testPose() {
        Pose3d desired = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(0, 0, 1));
        Vector<N3> axis = VecBuilder.fill(0, 0, 1);
        DoubleUnaryOperator f = (x) -> {
            Pose3d sample = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(axis, x));
            Twist3d t = desired.log(sample);
            return GeometryUtil.norm(t);
        };
        GoldenSectionSearch s = new GoldenSectionSearch(f, 1e-12, 100);
        assertEquals(1.0, s.solve(-Math.PI, Math.PI), 1e-12);
    }

    /** 8 us per solve on my laptop */
    @Test
    void testPosePerformance() {
        Pose3d desired = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(0, 0, 1));
        Vector<N3> axis = VecBuilder.fill(0, 0, 1);
        DoubleUnaryOperator f = (x) -> {
            Pose3d sample = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(axis, x));
            Twist3d t = desired.log(sample);
            return GeometryUtil.norm(t);
        };
        GoldenSectionSearch s = new GoldenSectionSearch(f, 1e-3, 100);

        int iterations = 100000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            s.solve(-Math.PI, Math.PI);
        }
        long finishTime = System.currentTimeMillis();

        Util.println("Golden section search on tangent norm");
        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
    }

    /**
     * See https://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf
     * 
     * This paper considers metrics in SO(3) that may be faster than the natural
     * metric.
     * 
     * Below we implement the simplest one, which it calls $\Phi_4$, the dot product
     * of quaternions.  It's about 3X faster than the natural metric.
     */
    @Test
    void testFasterPose() {
        Pose3d desired = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(0, 0, 1));
        Translation3d t0 = desired.getTranslation();
        Quaternion q0 = desired.getRotation().getQuaternion();
        Vector<N3> axis = VecBuilder.fill(0, 0, 1);
        DoubleUnaryOperator f = (x) -> {
            Pose3d sample = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(axis, x));
            Quaternion q1 = sample.getRotation().getQuaternion();
            Translation3d t1 = sample.getTranslation();
            double tnorm = t0.minus(t1).getNorm();
            double qnorm = 1 - Math.abs(q0.dot(q1));
            return tnorm + qnorm;
        };
        GoldenSectionSearch s = new GoldenSectionSearch(f, 1e-6, 100);
        assertEquals(1.0, s.solve(-Math.PI, Math.PI), 1e-6);
    }

    /** 2 us per solve, using the faster metric and 1e-3 tolerance. */
    @Test
    void testFasterPosePerformance() {
        Pose3d desired = new Pose3d(new Translation3d(1, 1, 1), new Rotation3d(0, 0, 1));
        Translation3d t0 = desired.getTranslation();
        Quaternion q0 = desired.getRotation().getQuaternion();
        Vector<N3> axis = VecBuilder.fill(0, 0, 1);
        DoubleUnaryOperator f = (x) -> {
            Quaternion q1 = new Rotation3d(axis, x).getQuaternion();
            Translation3d t1 = new Translation3d(1, 1, 1);
            double tnorm = t0.minus(t1).getNorm();
            double qnorm = 1 - Math.abs(q0.dot(q1));
            return tnorm + qnorm;
        };
        GoldenSectionSearch s = new GoldenSectionSearch(f, 1e-3, 100);

        int iterations = 200000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            s.solve(-Math.PI, Math.PI);
        }
        long finishTime = System.currentTimeMillis();

        Util.println("Golden section search on quaternion dot product");
        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
    }

}
