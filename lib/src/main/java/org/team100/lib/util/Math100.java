package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;

/**
 * Various math utilities.
 */
public class Math100 {
    private static final boolean DEBUG = false;
    private static final double EPSILON = 1e-6;
    // we just don't need very precise answers.
    private static final double kRootTolerance = 0.0001;

    /**
     * Returns the real solutions to the quadratic ax^2 + bx + c.
     */
    public static List<Double> solveQuadratic(double a, double b, double c) {
        double disc = b * b - 4 * a * c;

        if (epsilonEquals(disc, 0.0)) {
            return List.of(-b / (2 * a));
        } else if (disc > 0.0) {
            return List.of(
                    (-b + Math.sqrt(disc)) / (2 * a),
                    (-b - Math.sqrt(disc)) / (2 * a));
        } else {
            return new ArrayList<>();
        }
    }

    public static boolean epsilonEquals(double x, double y) {
        return epsilonEquals(x, y, EPSILON);
    }

    public static boolean epsilonEquals(double x, double y, double epsilon) {
        return Math.abs(x - y) < epsilon;
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static double interpolate(double a, double b, double x) {
        if (x == 0)
            return a;
        if (x == 1)
            return b;
        x = limit(x, 0.0, 1.0);
        if (x < 1e-12)
            return a;
        if (x > (1 - 1e-12))
            return b;
        return a + (b - a) * x;
    }

    private Math100() {
    }

    /**
     * Produce an Euler angle equivalent to x but closer to measurement; might be
     * outside [-pi,pi].
     */
    public static double getMinDistance(double measurement, double x) {
        return MathUtil.angleModulus(x - measurement) + measurement;
    }

    /**
     * One-dimensional root-finding. Unlike the 2d version below, this uses simple
     * bisection.
     * 
     * @param func            to be solved
     * @param x_0             lower bound
     * @param f_0             f(x_0)
     * @param x_1             upper bound
     * @param f_1             f(x_1)
     * @param tolerance       how close to zero do we need
     * @param iterations_left iterations to go
     * @return s parameter
     */
    public static double findRoot(
            DoubleUnaryOperator func,
            double x_0,
            double f_0,
            double x_1,
            double f_1,
            double tolerance,
            int iterations_left) {
        if (DEBUG)
            Util.printf("*************** i %d x_0 %.8f f_0 %.8f x_1 %.8f f_1 %.8f\n", iterations_left, x_0, f_0, x_1,
                    f_1);
        if (iterations_left < 0) {
            // ran out of time
            return 1.0;
        }
        if (Math.abs(f_0 - f_1) <= tolerance) {
            // failed to find a solution
            return 1.0;
        }
        if (Math.abs(f_0) < tolerance) {
            if (DEBUG)
                Util.println("left edge is the solution");
            return x_0;
        }
        if (Math.abs(f_1) < tolerance) {
            if (DEBUG)
                Util.println("right edge is the solution");
            return x_1;
        }
        final double s_guess = 0.5;

        double x_guess = (x_1 - x_0) * s_guess + x_0;
        double f_guess = func.applyAsDouble(x_guess);
        if (DEBUG)
            Util.printf("************* guess f(%.8f) = %.8f\n", x_guess, f_guess);

        if (Math.abs(f_guess) < tolerance) {
            if (DEBUG)
                Util.printf("guess %.8f less than tolerance %.8f\n", f_guess, tolerance);
            return s_guess;
        }

        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                    + (1.0 - s_guess) * findRoot(func, x_guess, f_guess, x_1, f_1, tolerance, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess * findRoot(func, x_0, f_0, x_guess, f_guess, tolerance, iterations_left - 1);
        }
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula
     * falsi technique. This is a pretty naive way to do root finding, but it's
     * usually faster than simple bisection while being robust in ways that e.g. the
     * Newton-Raphson method isn't.
     * 
     * @param func            The Function2d to take the root of.
     * @param x_0             x value of the lower bracket.
     * @param y_0             y value of the lower bracket.
     * @param f_0             value of 'func' at x_0, y_0 (passed in by caller to
     *                        save a call to 'func' during recursion)
     * @param x_1             x value of the upper bracket.
     * @param y_1             y value of the upper bracket.
     * @param f_1             value of 'func' at x_1, y_1 (passed in by caller to
     *                        save a call to 'func' during recursion)
     * @param iterations_left Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that
     *         corresponds to the (approximate) root.
     */
    public static double findRoot(
            DoubleBinaryOperator func,
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            int iterations_left) {

        if (iterations_left < 0) {
            return 1.0;
        }
        if (Math.abs(f_0 - f_1) <= kRootTolerance) {
            return 1.0;
        }
        double s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
        double x_guess = (x_1 - x_0) * s_guess + x_0;
        double y_guess = (y_1 - y_0) * s_guess + y_0;
        double f_guess = func.applyAsDouble(x_guess, y_guess);

        if (Math.abs(f_guess) < kRootTolerance) {
            // this is new as of dec 2023, why wasn't this here before?
            return s_guess;
        }

        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                    + (1.0 - s_guess) * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }
}
