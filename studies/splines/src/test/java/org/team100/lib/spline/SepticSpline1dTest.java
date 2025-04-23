package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.spline.SepticSpline1d.SplineException;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

/**
 * see
 * https://docs.google.com/spreadsheets/d/19WbkNaxcRGHwYwLH1pu9ER3qxZrsYqDlZTdV-cmOM0I
 * 
 */
public class SepticSpline1dTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.001;

    /** Start and end the same => freak out */
    @Test
    void testNaN() {
        assertThrows(
                SplineException.class,
                () -> SepticSpline1d.viaMatrix(Double.NaN, 0, 0, 0, 0, 0, 0, 0));
    }

    void scan(SepticSpline1d spline) {
        for (double t = 0; t <= 1.0; t += 0.001) {
            double x = spline.getPosition(t);
            double v = spline.getVelocity(t);
            double a = spline.getAcceleration(t);
            if (DEBUG)
                Util.printf("%12.6f  %12.6f  %12.6f  %12.6f\n", t, x, v, a);
        }
    }

    double scanV(SepticSpline1d spline) {
        double maxV = 0;
        for (double t = 0; t <= 1.0; t += 0.001) {
            double v = spline.getVelocity(t);
            maxV = Math.max(maxV, Math.abs(v));
        }
        return maxV;
    }

    double scanA(SepticSpline1d spline) {
        double maxA = 0;
        for (double t = 0; t <= 1.0; t += 0.001) {
            double a = spline.getAcceleration(t);
            maxA = Math.max(maxA, Math.abs(a));
        }
        return maxA;
    }

    @Test
    void testMax1() {
        SepticSpline1d spline = SepticSpline1d.viaMatrix(0, 1, 0, 0, 0, 0, 0, 0);
        // check initials
        assertEquals(0, spline.getPosition(0), kDelta);
        assertEquals(0, spline.getVelocity(0), kDelta);
        assertEquals(0, spline.getAcceleration(0), kDelta);
        assertEquals(0, spline.getJerk(0), kDelta);
        // check finals
        assertEquals(1, spline.getPosition(1), kDelta);
        assertEquals(0, spline.getVelocity(1), kDelta);
        assertEquals(0, spline.getAcceleration(1), kDelta);
        assertEquals(0, spline.getJerk(1), kDelta);
        // true values
        assertEquals(2.187, scanV(spline), kDelta);
        assertEquals(7.513, scanA(spline), kDelta);
        // what the spline thinks
        assertEquals(2.187, spline.maxV, kDelta);
        assertEquals(7.513, spline.maxA, kDelta);
    }

    @Test
    void testMax2() {
        // initial velocity
        SepticSpline1d spline = SepticSpline1d.viaMatrix(0, 1, 1, 0, 0, 0, 0, 0);
        // check initials
        assertEquals(0, spline.getPosition(0), kDelta);
        assertEquals(1, spline.getVelocity(0), kDelta);
        assertEquals(0, spline.getAcceleration(0), kDelta);
        assertEquals(0, spline.getJerk(0), kDelta);
        // check finals
        assertEquals(1, spline.getPosition(1), kDelta);
        assertEquals(0, spline.getVelocity(1), kDelta);
        assertEquals(0, spline.getAcceleration(1), kDelta);
        assertEquals(0, spline.getJerk(1), kDelta);
        scan(spline);
        // true values
        assertEquals(1.66, scanV(spline), kDelta);
        assertEquals(5.028, scanA(spline), kDelta);
        // what the spline thinks
        assertEquals(1.66, spline.maxV, kDelta);
        assertEquals(5.028, spline.maxA, kDelta);
    }

    @Test
    void testMax3() {
        SepticSpline1d spline = SepticSpline1d.viaMatrix(0, 1, 0, 0, 2, 0, 0, 0);
        // check initials
        assertEquals(0, spline.getPosition(0), kDelta);
        assertEquals(0, spline.getVelocity(0), kDelta);
        assertEquals(2, spline.getAcceleration(0), kDelta);
        assertEquals(0, spline.getJerk(0), kDelta);
        // check finals
        assertEquals(1, spline.getPosition(1), kDelta);
        assertEquals(0, spline.getVelocity(1), kDelta);
        assertEquals(0, spline.getAcceleration(1), kDelta);
        assertEquals(0, spline.getJerk(1), kDelta);
        // true values
        assertEquals(2.071, scanV(spline), kDelta);
        assertEquals(6.850, scanA(spline), kDelta);
        // what the spline thinks
        assertEquals(2.071, spline.maxV, kDelta);
        assertEquals(6.850, spline.maxA, kDelta);
    }

    @Test
    void testMax4() {
        SepticSpline1d spline = SepticSpline1d.viaMatrix(0, 1, 0, 0, 0, 0, 50, 0);
        // check initials
        assertEquals(0, spline.getPosition(0), kDelta);
        assertEquals(0, spline.getVelocity(0), kDelta);
        assertEquals(0, spline.getAcceleration(0), kDelta);
        assertEquals(50, spline.getJerk(0), kDelta);
        // check finals
        assertEquals(1, spline.getPosition(1), kDelta);
        assertEquals(0, spline.getVelocity(1), kDelta);
        assertEquals(0, spline.getAcceleration(1), kDelta);
        assertEquals(0, spline.getJerk(1), kDelta);
        scan(spline);
        // true values
        assertEquals(2.085, scanV(spline), kDelta);
        assertEquals(7.054, scanA(spline), kDelta);
        // what the spline thinks
        assertEquals(2.085, spline.maxV, kDelta);
        assertEquals(7.054, spline.maxA, kDelta);
    }

    /** Look at an example */
    @Test
    void testSample() {
        // an example from 0 to 1 with zero first, second, and third derivatives at the
        // ends.
        double vLim = 2;
        double aLim = 6;
        SepticSpline1d spline = SepticSpline1d.viaMatrix(0, 1, 0, 0, 0, 0, 0, 0);
        // double scale = 0.5;
        double vScale = spline.maxV / vLim;
        double aScale = Math.sqrt(spline.maxA) / Math.sqrt(aLim);
        double duration = Math.max(vScale, aScale);
        // natural v is slightly too high
        assertEquals(1.094, vScale, kDelta);
        // natural a is also slightly too high
        assertEquals(1.119, aScale, kDelta);
        assertEquals(1.119, duration, kDelta);

        for (double timeSec = 0; timeSec <= duration; timeSec += duration * 0.01) {
            double param = timeSec / duration;
            double x = spline.getPosition(param);
            double v = spline.getVelocity(param) / duration;
            double a = spline.getAcceleration(param) / (duration * duration);
            double j = spline.getJerk(param) / (duration * duration * duration);
            double s = spline.getSnap(param) / (duration * duration * duration * duration);
            double c = spline.getCrackle(param) / (duration * duration * duration * duration * duration);
            double p = spline.getPop(param) / (duration * duration * duration * duration * duration * duration);
            if (DEBUG)
                Util.printf("%12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f\n",
                        timeSec, x, v, a, j, s, c, p);
        }
        // find the maxima
        //
        // maximum velocity is at the accel zero which is the same as the pop zero
        // -720b/540a
        // ... is it really always the same?
        //
        assertEquals(0, spline.getPop(0.5), kDelta);
        double t05 = -720 * spline.b / (5040 * spline.a);
        assertEquals(0.5, t05, kDelta);
        double maxV = spline.getVelocity(t05);
        assertEquals(2.187, maxV, kDelta);
        assertEquals(2.187, spline.maxV, kDelta);
        //
        // max accel is at the jerk zero which is the same as the crackle zero
        // solve the quadratic 2520at^2 + 720bt + 120c = 0
        List<Double> zeros = Math100.solveQuadratic(2520 * spline.a, 720 * spline.b, 120 * spline.c);
        assertEquals(2, zeros.size());
        assertEquals(0.276, zeros.get(0), kDelta);
        assertEquals(0.724, zeros.get(1), kDelta);
        double maxA = spline.getAcceleration(zeros.get(0));
        assertEquals(7.513, maxA, kDelta);
        assertEquals(7.513, spline.maxA, kDelta);

    }

}
