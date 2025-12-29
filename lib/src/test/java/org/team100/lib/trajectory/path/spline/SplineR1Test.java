package org.team100.lib.trajectory.path.spline;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N6;

/**
 * see
 * https://docs.google.com/spreadsheets/d/19WbkNaxcRGHwYwLH1pu9ER3qxZrsYqDlZTdV-cmOM0I
 * 
 */
public class SplineR1Test {
    private static final boolean DEBUG = false;

    /**
     * Making the end derivatives the average, with zero second derivative, makes
     * the spline always just a straight line
     */
    @Test
    void testEnds() {
        SplineR1 spline = SplineR1.get(0, 1, 1, 1, 0, 0);
        show(spline);
    }

    /** Look at an example */
    @Test
    void testSample() {
        SplineR1 spline = SplineR1.viaMatrix(0, 1, 0, 0, 0, 0);
        show(spline);
    }

    private void show(SplineR1 spline) {
        if (DEBUG)
            System.out.println("t, x, v, a, j");
        for (double t = 0; t <= 1; t += 0.01) {
            double x = spline.getPosition(t);
            double v = spline.getVelocity(t);
            double a = spline.getAcceleration(t);
            double j = spline.getJerk(t);
            if (DEBUG)
                System.out.printf("%8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n",
                        t, x, v, a, j);
        }
    }

    @Test
    void testCoefficients() {
        // how to get the quintic spline coefficients.
        // see https://janhuenermann.com/paper/spline2020.pdf
        //
        // the quintic spline and its derivatives are:
        //
        // x = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5
        // v = c1 + 2 c2 t + 3 c3 t^2 + 4 c4 t^3 + 5 c5 t^4
        // a = 2 c2 + 6 c3 t + 12 c4 t^2 + 20 c5 t^3
        //
        // as a matrix, first three rows are for t=0,
        // second three rows are t=1
        // (x0;v0;a0;x1;v1;a1) = A [c0; c1; c2; c3; c4; c5]
        Matrix<N6, N6> A = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 2, 0, 0, 0, //
                1, 1, 1, 1, 1, 1, //
                0, 1, 2, 3, 4, 5, //
                0, 0, 2, 6, 12, 20);
        // so to get the c vector we just invert the matrix
        Matrix<N6, N6> Ainv = A.inv();
        if (DEBUG)
            System.out.printf("%s\n", Ainv);

    }

}
