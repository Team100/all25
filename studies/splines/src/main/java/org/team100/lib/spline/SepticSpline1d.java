package org.team100.lib.spline;

import org.team100.lib.util.Util;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N8;

/** 7th order spline allows control of jerk. */
public class SepticSpline1d {
    public static class SplineException extends RuntimeException {
        public SplineException(String msg) {
            super(msg);
        }

        public SplineException(RuntimeException ex) {
            super(ex);
        }
    }

    final double a;
    final double b;
    final double c;
    final double d;
    final double e;
    final double f;
    final double g;
    final double h;
    // max of the underlying spline
    public final double maxV;
    public final double maxA;

    private SepticSpline1d(double a, double b, double c, double d, double e, double f, double g, double h) {
        if (Double.isNaN(a))
            throw new SplineException("a");
        if (Double.isNaN(b))
            throw new SplineException("b");
        if (Double.isNaN(c))
            throw new SplineException("c");
        if (Double.isNaN(d))
            throw new SplineException("d");
        if (Double.isNaN(e))
            throw new SplineException("e");
        if (Double.isNaN(f))
            throw new SplineException("f");
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.e = e;
        this.f = f;
        this.g = g;
        this.h = h;
        maxV = scanV();
        maxA = scanA();
    }

    public static SepticSpline1d viaMatrix(
            double x0,
            double x1,
            double dx0,
            double dx1,
            double ddx0,
            double ddx1,
            double dddx0,
            double dddx1) {
        // how to get the quintic spline coefficients.
        // see https://janhuenermann.com/paper/spline2020.pdf
        //
        // the septic spline and its derivatives are:
        //
        // x = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5 + c6 t^6 + c7 t^7
        // v = c1 + 2 c2 t + 3 c3 t^2 + 4 c4 t^3 + 5 c5 t^4 + 6 c6 t^5 + 7 c7 t^6
        // a = 2 c2 + 6 c3 t + 12 c4 t^2 + 20 c5 t^3 + 30 c6 t^4 + 42 c7 t^5
        // j = 6 c3 + 24 c4 t + 60 ct t^2 + 120 c6 t^3 + 210 c7 t^4
        //
        // as a matrix, first four rows are for t=0,
        // second four rows are t=1
        // (x0;v0;a0;j0;x1;v1;a1;j1) = A [c0; c1; c2; c3; c4; c5; c6; c7]
        Matrix<N8, N8> A = MatBuilder.fill(Nat.N8(), Nat.N8(), //
                1, 0, 0, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, 0, 0, //
                0, 0, 2, 0, 0, 0, 0, 0, //
                0, 0, 0, 6, 0, 0, 0, 0, //
                1, 1, 1, 1, 1, 1, 1, 1, //
                0, 1, 2, 3, 4, 5, 6, 7, //
                0, 0, 2, 6, 12, 20, 30, 42, //
                0, 0, 0, 6, 24, 60, 120, 210);
        // so to get the c vector we just invert the matrix
        Matrix<N8, N8> Ainv = A.inv();
        Vector<N8> x = VecBuilder.fill(x0, dx0, ddx0, dddx0, x1, dx1, ddx1, dddx1);
        Matrix<N8, N1> c = Ainv.times(x);
        try {
            return new SepticSpline1d(
                    c.get(7, 0),
                    c.get(6, 0),
                    c.get(5, 0),
                    c.get(4, 0),
                    c.get(3, 0),
                    c.get(2, 0),
                    c.get(1, 0),
                    c.get(0, 0));
        } catch (SplineException ex) {
            Util.warnf("Spline error x0 %f x1 %f dx0 %f dx1 %f ddx0 %f ddx1 %f dddx0 %f dddx1 %f\n",
                    x0, x1, dx0, dx1, ddx0, ddx1, dddx0, dddx1);
            throw new SplineException(ex);
        }
    }

    /**
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    public double getPosition(double t) {
        return a * t * t * t * t * t * t * t +
                b * t * t * t * t * t * t +
                c * t * t * t * t * t +
                d * t * t * t * t +
                e * t * t * t +
                f * t * t +
                g * t +
                h;
    }

    /**
     * @return rate of change of position with respect to parameter, i.e. ds/dt
     */
    public double getVelocity(double t) {
        return 7 * a * t * t * t * t * t * t +
                6 * b * t * t * t * t * t +
                5 * c * t * t * t * t +
                4 * d * t * t * t +
                3 * e * t * t +
                2 * f * t +
                g;
    }

    /**
     * @return acceleration of position with respect to parameter, i.e. d^2s/dt^2
     */
    public double getAcceleration(double t) {
        return 42.0 * a * t * t * t * t * t +
                30.0 * b * t * t * t * t +
                20.0 * c * t * t * t +
                12.0 * d * t * t +
                6.0 * e * t +
                2.0 * f;
    }

    /**
     * @return jerk of position with respect to parameter, i.e. d^3s/dt^3.
     */
    public double getJerk(double t) {
        return 210.0 * a * t * t * t * t +
                120.0 * b * t * t * t +
                60.0 * c * t * t +
                24.0 * d * t +
                6.0 * e;
    }

    /**
     * @return snap of position with respect to parameter, i.e. d^4s/dt^4.
     */
    public double getSnap(double t) {
        return 840.0 * a * t * t * t +
                360.0 * b * t * t +
                120.0 * c * t +
                24.0 * d;
    }

    public double getCrackle(double t) {
        return 2520.0 * a * t * t +
                720.0 * b * t +
                120.0 * c;
    }

    public double getPop(double t) {
        return 5040.0 * a * t +
                720.0 * b;
    }

    double scanV() {
        double maxV = 0;
        for (double t = 0; t <= 1.0; t += 0.001) {
            double v = getVelocity(t);
            maxV = Math.max(maxV, Math.abs(v));
        }
        return maxV;
    }

    double scanA() {
        double maxA = 0;
        for (double t = 0; t <= 1.0; t += 0.001) {
            double a = getAcceleration(t);
            maxA = Math.max(maxA, Math.abs(a));
        }
        return maxA;
    }

}
