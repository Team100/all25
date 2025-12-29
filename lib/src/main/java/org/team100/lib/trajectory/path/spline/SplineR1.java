package org.team100.lib.trajectory.path.spline;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;

/**
 * One-dimensional quintic spline, representing five derivatives of position.
 * 
 * The "s" parameter here is not time, its just a parameter.
 */
public class SplineR1 {
    /** crackle */
    private final double a;
    /** snap */
    private final double b;
    /** jerk */
    private final double c;
    /** acceleration */
    private final double d;
    /** velocity */
    private final double e;
    /** position */
    private final double f;

    private SplineR1(double a, double b, double c, double d, double e, double f) {
        if (Double.isNaN(a))
            throw new IllegalArgumentException();
        if (Double.isNaN(b))
            throw new IllegalArgumentException();
        if (Double.isNaN(c))
            throw new IllegalArgumentException();
        if (Double.isNaN(d))
            throw new IllegalArgumentException();
        if (Double.isNaN(e))
            throw new IllegalArgumentException();
        if (Double.isNaN(f))
            throw new IllegalArgumentException();
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.e = e;
        this.f = f;
    }

    public static SplineR1 get(
            double x0,
            double x1,
            double dx0,
            double dx1,
            double ddx0,
            double ddx1) {
        if (Double.isNaN(x0))
            throw new IllegalArgumentException();
        if (Double.isNaN(x1))
            throw new IllegalArgumentException();
        if (Double.isNaN(dx0))
            throw new IllegalArgumentException();
        if (Double.isNaN(dx1))
            throw new IllegalArgumentException();
        if (Double.isNaN(ddx0))
            throw new IllegalArgumentException();
        if (Double.isNaN(ddx1))
            throw new IllegalArgumentException();

        // for derivation of the coefficients, see
        // https://janhuenermann.com/paper/spline2020.pdf

        double a = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
        double b = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
        double c = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
        double d = 0.5 * ddx0;
        double e = dx0;
        double f = x0;
        return new SplineR1(a, b, c, d, e, f);
    }

    public static SplineR1 viaMatrix(
            double x0,
            double x1,
            double dx0,
            double dx1,
            double ddx0,
            double ddx1) {
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
        Vector<N6> x = VecBuilder.fill(x0, dx0, ddx0, x1, dx1, ddx1);
        Matrix<N6, N1> c = Ainv.times(x);
        return new SplineR1(
                c.get(5, 0),
                c.get(4, 0),
                c.get(3, 0),
                c.get(2, 0),
                c.get(1, 0),
                c.get(0, 0));
    }

    SplineR1 addCoefs(SplineR1 other) {
        double aa = a + other.a;
        double bb = b + other.b;
        double cc = c + other.c;
        double dd = d + other.d;
        double ee = e + other.e;
        double ff = f + other.f;
        return new SplineR1(aa, bb, cc, dd, ee, ff);
    }

    /**
     * @param s ranges from 0 to 1
     * @return the point on the spline for that s value
     */
    public double getPosition(double s) {
        return a * s * s * s * s * s + b * s * s * s * s + c * s * s * s + d * s * s + e * s + f;
    }

    /**
     * @return rate of change of position with respect to parameter, i.e. dq/ds
     */
    public double getVelocity(double s) {
        return 5 * a * s * s * s * s + 4 * b * s * s * s + 3 * c * s * s + 2 * d * s + e;
    }

    /**
     * @return acceleration of position with respect to parameter, i.e. d^2q/ds^2
     */
    public double getAcceleration(double s) {
        return 20 * a * s * s * s + 12 * b * s * s + 6 * c * s + 2 * d;
    }

    /**
     * @return jerk of position with respect to parameter, i.e. d^3q/ds^3.
     */
    public double getJerk(double s) {
        return 60 * a * s * s + 24 * b * s + 6 * c;
    }

    /**
     * @return snap of position with respect to parameter, i.e. d^4q/ds^4.
     */
    public double getSnap(double s) {
        return 120 * a * s + 24 * b;
    }

    @Override
    public String toString() {
        return String.format("Spline1d [%6.2f %6.2f %6.2f] - [%6.2f %6.2f %6.2f]",
                getPosition(0), getVelocity(0), getAcceleration(0),
                getPosition(1), getVelocity(1), getAcceleration(1));
    }

}