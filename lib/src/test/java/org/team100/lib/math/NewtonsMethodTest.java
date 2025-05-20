package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Function;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class NewtonsMethodTest {
    /** Multivariate scalar function, f(x) = norm(x)^2 */
    @Test
    void test1() {
        Function<Vector<N2>, Vector<N1>> f = x -> VecBuilder.fill(Math.pow(x.normF(), 2));
        Vector<N2> x = VecBuilder.fill(1, 0.5);
        Vector<N1> Y = f.apply(x);
        assertEquals(1.25, Y.get(0), 1e-9);

        Matrix<N1, N2> j = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N1(), f, x);
        assertEquals(2, j.get(0, 0), 1e-9);
        assertEquals(1, j.get(0, 1), 1e-9);

        {
            // pseudoinverse
            Matrix<N2, N1> jInv = new Matrix<>(j.getStorage().pseudoInverse());
            assertEquals(0.4, jInv.get(0, 0), 1e-9);
            assertEquals(0.2, jInv.get(1, 0), 1e-9);
            Vector<N2> dx = new Vector<>(jInv.times(Y));
            Vector<N2> x2 = x.minus(dx);
            assertEquals(0.5, x2.get(0), 1e-9);
            assertEquals(0.25, x2.get(1), 1e-9);
        }
        {
            // solve (does not work)
            // Vector<N2> dx = new Vector<>(j.solve(Y.times(-1)));
            // Vector<N2> x2 = x.plus(dx);
            // assertEquals(-100, x2.get(0), 1e-9);
            // assertEquals(-100, x2.get(1), 1e-9);
        }

    }

    /** Multivariate vector function, f(x) = x */
    @Test
    void test2() {
        Function<Vector<N2>, Vector<N2>> f = x -> x;
        Vector<N2> x = VecBuilder.fill(1, 1);
        Vector<N2> Y = f.apply(x);
        assertEquals(1, Y.get(0), 1e-9);
        assertEquals(1, Y.get(1), 1e-9);

        // jacobian is identity
        Matrix<N2, N2> j = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N2(), f, x);
        assertEquals(1, j.get(0, 0), 1e-9);
        assertEquals(0, j.get(0, 1), 1e-9);
        assertEquals(0, j.get(1, 0), 1e-9);
        assertEquals(1, j.get(1, 1), 1e-9);
        // in this case it's invertible; inverse is identity.
        Matrix<N2, N2> jInverse = j.inv();
        assertEquals(1, jInverse.get(0, 0), 1e-9);
        assertEquals(0, jInverse.get(0, 1), 1e-9);
        assertEquals(0, jInverse.get(1, 0), 1e-9);
        assertEquals(1, jInverse.get(1, 1), 1e-9);
        {
            // pseudoinverse
            Matrix<N2, N2> jInv = new Matrix<>(j.getStorage().pseudoInverse());
            assertEquals(1, jInv.get(0, 0), 1e-9);
            assertEquals(0, jInv.get(0, 1), 1e-9);
            assertEquals(0, jInv.get(1, 0), 1e-9);
            assertEquals(1, jInv.get(1, 1), 1e-9);

            // since f is linear the Newton method works perfectly:
            Vector<N2> dx = new Vector<>(jInv.times(Y));
            Vector<N2> x2 = x.minus(dx);
            assertEquals(0, x2.get(0), 1e-9);
            assertEquals(0, x2.get(1), 1e-9);
        }
        {
            // solve also works
            Vector<N2> dx = new Vector<>(j.solve(Y.times(-1)));
            Vector<N2> x2 = x.plus(dx);
            assertEquals(0, x2.get(0), 1e-9);
            assertEquals(0, x2.get(1), 1e-9);
        }
    }

    @Test
    void test3() {
        // 2d RR arm example, f translation as a function of joint angles q
        Function<Vector<N2>, Vector<N2>> f = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)));

        // desired position
        // q0 should be pi/6 or 0.524, q1 should be 2pi/3 or 2.094
        Vector<N2> Xd = VecBuilder.fill(0, 1);

        // initial joint angles
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // initial position
        Vector<N2> X0 = f.apply(q0);
        assertEquals(1, X0.get(0), 1e-9);
        assertEquals(1, X0.get(1), 1e-9);
        // jacobian at q0
        Matrix<N2, N2> j0 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N2(), f, q0);
        assertEquals(-1, j0.get(0, 0), 1e-9);
        assertEquals(-1, j0.get(0, 1), 1e-9);
        assertEquals(1, j0.get(1, 0), 1e-9);
        assertEquals(0, j0.get(1, 1), 1e-9);
        Vector<N2> dq1 = new Vector<>(j0.solve(Xd.minus(X0)));

        Vector<N2> q1 = q0.plus(dq1);
        assertEquals(0, q1.get(0), 1e-9);
        assertEquals(2.571, q1.get(1), 1e-3);

        Matrix<N2, N2> j1 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N2(), f, q1);
        Vector<N2> X1 = f.apply(q1);
        Vector<N2> dq2 = new Vector<>(j1.solve(Xd.minus(X1)));
        Vector<N2> q2 = q1.plus(dq2);
        assertEquals(0.707, q2.get(0), 1e-3);
        assertEquals(2.158, q2.get(1), 1e-3);

        Matrix<N2, N2> j2 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N2(), f, q2);
        Vector<N2> X2 = f.apply(q2);
        Vector<N2> dq3 = new Vector<>(j2.solve(Xd.minus(X2)));
        Vector<N2> q3 = q2.plus(dq3);
        assertEquals(0.5, q3.get(0), 1e-3);
        assertEquals(2.121, q3.get(1), 1e-3);

        // by step 3 we have the answer
        Matrix<N2, N2> j3 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N2(), f, q3);
        Vector<N2> X3 = f.apply(q3);
        Vector<N2> dq4 = new Vector<>(j3.solve(Xd.minus(X3)));
        Vector<N2> q4 = q3.plus(dq4);
        assertEquals(0.524, q4.get(0), 1e-3);
        assertEquals(2.094, q4.get(1), 1e-3);
    }

    @Test
    void test4() {
        // Same as above but including end angle. note that end angle
        // can't be specified independently of position. so what happens?
        // when we specify the right angle, it speeds up the solver a bit.
        Function<Vector<N2>, Vector<N3>> f = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)),
                q.get(0) + q.get(1));

        // desired position
        // q0 should be pi/6 or 0.524, q1 should be 2pi/3 or 2.094
        // the resulting end-angle is 5pi/6
        Vector<N3> Xd = VecBuilder.fill(0, 1, 2.618);

        // initial joint angles
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // initial position
        Vector<N3> X0 = f.apply(q0);
        assertEquals(1, X0.get(0), 1e-9);
        assertEquals(1, X0.get(1), 1e-9);
        // jacobian at q0
        Matrix<N3, N2> j0 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), f, q0);
        assertEquals(-1, j0.get(0, 0), 1e-9);
        assertEquals(-1, j0.get(0, 1), 1e-9);
        assertEquals(1, j0.get(1, 0), 1e-9);
        assertEquals(0, j0.get(1, 1), 1e-9);
        Vector<N2> dq1 = new Vector<>(j0.solve(Xd.minus(X0)));

        Vector<N2> q1 = q0.plus(dq1);
        assertEquals(0, q1.get(0), 1e-9);
        assertEquals(2.594, q1.get(1), 1e-3);

        Matrix<N3, N2> j1 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), f, q1);
        Vector<N3> X1 = f.apply(q1);
        Vector<N2> dq2 = new Vector<>(j1.solve(Xd.minus(X1)));
        Vector<N2> q2 = q1.plus(dq2);
        assertEquals(0.547, q2.get(0), 1e-3);
        assertEquals(2.126, q2.get(1), 1e-3);

        Matrix<N3, N2> j2 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), f, q2);
        Vector<N3> X2 = f.apply(q2);
        Vector<N2> dq3 = new Vector<>(j2.solve(Xd.minus(X2)));
        Vector<N2> q3 = q2.plus(dq3);
        assertEquals(0.522, q3.get(0), 1e-3);
        assertEquals(2.096, q3.get(1), 1e-3);

        // by step 3 we have the answer
        Matrix<N3, N2> j3 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), f, q3);
        Vector<N3> X3 = f.apply(q3);
        Vector<N2> dq4 = new Vector<>(j3.solve(Xd.minus(X3)));
        Vector<N2> q4 = q3.plus(dq4);
        assertEquals(0.524, q4.get(0), 1e-3);
        assertEquals(2.094, q4.get(1), 1e-3);
    }

    @Test
    void test5() {
        // Same as above but with an inconsistent end angle: the solver compromises
        // since it's a least-squares thing
        Function<Vector<N2>, Vector<N3>> f = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)),
                q.get(0) + q.get(1));

        // desired position
        // q0 should be pi/6 or 0.524, q1 should be 2pi/3 or 2.094
        // the resulting end-angle is 5pi/6, so this is wrong.
        Vector<N3> Xd = VecBuilder.fill(0, 1, 2);

        // initial joint angles
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // initial position
        Vector<N3> X0 = f.apply(q0);
        assertEquals(1, X0.get(0), 1e-9);
        assertEquals(1, X0.get(1), 1e-9);
        // jacobian at q0
        Matrix<N3, N2> j0 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), f, q0);
        assertEquals(-1, j0.get(0, 0), 1e-9);
        assertEquals(-1, j0.get(0, 1), 1e-9);
        assertEquals(1, j0.get(1, 0), 1e-9);
        assertEquals(0, j0.get(1, 1), 1e-9);
        Vector<N2> dq1 = new Vector<>(j0.solve(Xd.minus(X0)));

        Vector<N2> q1 = q0.plus(dq1);
        assertEquals(0, q1.get(0), 1e-9);
        assertEquals(2.285, q1.get(1), 1e-3);

        Matrix<N3, N2> j1 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), f, q1);
        Vector<N3> X1 = f.apply(q1);
        Vector<N2> dq2 = new Vector<>(j1.solve(Xd.minus(X1)));
        Vector<N2> q2 = q1.plus(dq2);
        assertEquals(0.234, q2.get(0), 1e-3);
        assertEquals(2.035, q2.get(1), 1e-3);

        Matrix<N3, N2> j2 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), f, q2);
        Vector<N3> X2 = f.apply(q2);
        Vector<N2> dq3 = new Vector<>(j2.solve(Xd.minus(X2)));
        Vector<N2> q3 = q2.plus(dq3);
        assertEquals(0.317, q3.get(0), 1e-3);
        assertEquals(1.962, q3.get(1), 1e-3);

        // by step 3 we have the answer
        Matrix<N3, N2> j3 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), f, q3);
        Vector<N3> X3 = f.apply(q3);
        Vector<N2> dq4 = new Vector<>(j3.solve(Xd.minus(X3)));
        Vector<N2> q4 = q3.plus(dq4);
        assertEquals(0.344, q4.get(0), 1e-3);
        assertEquals(1.938, q4.get(1), 1e-3);
    }

    @Test
    void test6() {
        // case from test3 but using the solver class
        Function<Vector<N2>, Vector<N2>> f = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)));
        NewtonsMethod<N2, N2> s = new NewtonsMethod<>(Nat.N2(), Nat.N2(), f, 1e-3, 10);
        Vector<N2> Xd = VecBuilder.fill(0, 1);
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        Vector<N2> x = s.solve(q0, Xd);
        assertEquals(0.524, x.get(0), 1e-3);
        assertEquals(2.094, x.get(1), 1e-3);
    }

    /** 5 us per solve */
    @Test
    void test7() {
        // performance
        Function<Vector<N2>, Vector<N2>> f = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)));
        NewtonsMethod<N2, N2> s = new NewtonsMethod<>(Nat.N2(), Nat.N2(), f, 1e-3, 10);
        Vector<N2> Xd = VecBuilder.fill(0, 1);
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        int iterations = 100000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            s.solve(q0, Xd);
        }
        long finishTime = System.currentTimeMillis();
        Util.println("Newton's method for RR arm");
        Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
        Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);

    }
}
