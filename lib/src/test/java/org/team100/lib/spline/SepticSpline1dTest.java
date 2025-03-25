package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

/**
 * see
 * https://docs.google.com/spreadsheets/d/19WbkNaxcRGHwYwLH1pu9ER3qxZrsYqDlZTdV-cmOM0I
 * 
 */
public class SepticSpline1dTest {

    private static final boolean DEBUG = true;
    private static final double kDelta = 0.001;

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
