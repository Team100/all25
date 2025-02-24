package org.team100.lib.spline;

import java.util.List;
import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Static utility methods for splines. */
public class SplineUtil {
    private static final boolean DEBUG = true;
    private static final double kEpsilon = 1e-5;
    // TODO: find a good step size.
    private static final double kStepSize = 1.0;
    private static final double kMinDelta = 0.001;
    private static final int kMaxIterations = 100;

    /**
     * Makes optimization code a little more readable
     */
    static class ControlPoint {
        double ddx;
        double ddy;
    }

    /**
     * True if adjacent spline endpoints have (nearly) identical derivative terms.
     */
    public static boolean verifyC1(List<HolonomicSpline> splines) {
        if (splines.size() < 2)
            return true;
        for (int i = 0; i < splines.size() - 1; ++i) {
            HolonomicSpline s0 = splines.get(i);
            HolonomicSpline s1 = splines.get(i + 1);
            if (!MathUtil.isNear(s0.dx(1), s1.dx(0), 1e-6)) {
                if (DEBUG)
                    Util.printf("bad x C1 %f %f\n", s0.dx(1), s1.dx(0));
                return false;
            }
            if (!MathUtil.isNear(s0.dy(1), s1.dy(0), 1e-6)) {
                if (DEBUG)
                    Util.printf("bad y C1 %f %f\n", s0.dy(1), s1.dy(0));
                return false;
            }
            if (!MathUtil.isNear(s0.dtheta(1), s1.dtheta(0), 1e-6)) {
                if (DEBUG)
                    Util.printf("bad theta C1 %f %f\n", s0.dtheta(1), s1.dtheta(0));
                return false;
            }
        }
        return true;
    }

    /**
     * True if adjacent spline endpoints have (nearly) identical second-derivative
     * terms.
     */
    public static boolean verifyC2(List<HolonomicSpline> splines) {
        if (splines.size() < 2)
            return true;
        for (int i = 0; i < splines.size() - 1; ++i) {
            HolonomicSpline s0 = splines.get(i);
            HolonomicSpline s1 = splines.get(i + 1);
            if (!MathUtil.isNear(s0.ddx(1), s1.ddx(0), 1e-6)) {
                if (DEBUG)
                    Util.printf("bad x C2 %f %f\n", s0.ddx(1), s1.ddx(0));
                return false;
            }
            if (!MathUtil.isNear(s0.ddy(1), s1.ddy(0), 1e-6)) {
                if (DEBUG)
                    Util.printf("bad y C2 %f %f\n", s0.ddy(1), s1.ddy(0));
                return false;
            }
            if (!MathUtil.isNear(s0.ddtheta(1), s1.ddtheta(0), 1e-6)) {
                if (DEBUG)
                    Util.printf("bad theta C2 %f %f\n", s0.ddtheta(1), s1.ddtheta(0));
                return false;
            }
        }
        return true;
    }

    /**
     * True if adjacent spline endpoints have (nearly) identical third-derivative
     * terms.
     */
    public static boolean verifyC3(List<HolonomicSpline> splines) {
        if (splines.size() < 2)
            return true;
        for (int i = 0; i < splines.size() - 1; ++i) {
            HolonomicSpline s0 = splines.get(i);
            HolonomicSpline s1 = splines.get(i + 1);
            if (!MathUtil.isNear(s0.dddx(1), s1.dddx(0), 1e-6)) {
                if (DEBUG)
                    Util.printf("bad x C3 %f %f\n", s0.dddx(1), s1.dddx(0));
                return false;
            }
            if (!MathUtil.isNear(s0.dddy(1), s1.dddy(0), 1e-6)) {
                if (DEBUG)
                    Util.printf("bad y C3 %f %f\n", s0.dddy(1), s1.dddy(0));
                return false;
            }
            if (!MathUtil.isNear(s0.dddtheta(1), s1.dddtheta(0), 1e-6)) {
                if (DEBUG)
                    Util.printf("bad theta C3 %f %f\n", s0.dddtheta(1), s1.dddtheta(0));
                return false;
            }
        }
        return true;
    }

    /**
     * Force derivatives to be the same at joints, by choosing the average.
     */
    public static void forceC1(List<HolonomicSpline> splines) {
        if (splines.size() < 2)
            return;
        for (int i = 0; i < splines.size() - 1; ++i) {
            HolonomicSpline s0 = splines.get(i);
            HolonomicSpline s1 = splines.get(i + 1);

            // the derivatives need to be scaled by the distance of each spline
            // since they are derivatives with respect to the parameter,
            // not with respect to distance.
            double d0 = Math.hypot(s0.x(1) - s0.x(0), s0.y(1) - s0.y(0));
            double d1 = Math.hypot(s1.x(1) - s1.x(0), s1.y(1) - s1.y(0));
            if (DEBUG)
                Util.printf("d0 %f d1 %f\n", d0, d1);
            // // note, sometimes these can be zero
            if (d0 < 1e-3 || d1 < 1e-3)
                return;

            double s0dx1 = s0.dx(1);
            if (DEBUG)
                Util.printf("s0dx1 %f\n", s0dx1);
            double s1dx0 = s1.dx(0);
            if (DEBUG)
                Util.printf("s1dx0 %f\n", s1dx0);
            double meanDx = 0.5 * (s0dx1 / d0 + s1dx0 / d1);
            if (DEBUG)
                Util.printf("mean dx %f\n", meanDx);
            double s0dy1 = s0.dy(1);
            double s1dy0 = s1.dy(0);
            double meanDy = 0.5 * (s0dy1 / d0 + s1dy0 / d1);
            double s0dtheta1 = s0.dtheta(1);
            double s1dtheta0 = s1.dtheta(0);
            double meanDtheta = 0.5 * (s0dtheta1 / d0 + s1dtheta0 / d1);
            splines.set(
                    i,
                    s0.replaceFirstDerivatives(
                            s0.dx(0), meanDx * d0,
                            s0.dy(0), meanDy * d0,
                            s0.dtheta(0), meanDtheta * d0));
            splines.set(
                    i + 1,
                    s1.replaceFirstDerivatives(
                            meanDx * d1, s1.dx(1),
                            meanDy * d1, s1.dy(1),
                            meanDtheta * d1, s1.dtheta(1)));

        }
    }

    /**
     * Finds the optimal second derivative values for a set of splines to reduce the
     * sum of the change in curvature squared over the path
     *
     * @param splines the list of splines to optimize
     * @return the final sumDCurvature2
     */
    public static double optimizeSpline(List<HolonomicSpline> splines) {
        int count = 0;
        double prev = sumDCurvature2(splines);
        while (count < kMaxIterations) {
            runOptimizationIteration(splines);
            double current = sumDCurvature2(splines);
            if (prev - current < kMinDelta)
                return current;
            prev = current;
            count++;
        }
        Util.warn("Spline optimization failed");
        return prev;
    }

    /**
     * @return integral of dCurvature^2 over the length of multiple splines
     */
    static double sumDCurvature2(List<HolonomicSpline> splines) {
        double sum = 0;
        for (HolonomicSpline s : splines) {
            sum += s.sumDCurvature2();
        }
        if (Double.isNaN(sum))
            throw new IllegalArgumentException();
        return sum;
    }

    /**
     * Runs a single optimization iteration, using Newton's method.
     */
    static void runOptimizationIteration(List<HolonomicSpline> splines) {
        // can't optimize anything with less than 2 splines
        if (splines.size() <= 1) {
            // Util.warn("runOptimizationIteration: nothing to optimize");
            return;
        }

        ControlPoint[] controlPoints = new ControlPoint[splines.size() - 1];

        double magnitude = getControlPoints(splines, controlPoints);

        magnitude = Math.sqrt(magnitude);
        if (Double.isNaN(magnitude))
            throw new IllegalArgumentException();

        // minimize along the direction of the gradient
        // first calculate 3 points along the direction of the gradient

        // middle point is at the current location
        Translation2d p2 = new Translation2d(0, sumDCurvature2(splines));

        // first point is offset from the middle location by -stepSize
        for (int i = 0; i < splines.size() - 1; ++i) {
            backwards(splines, controlPoints, magnitude, i);
        }

        // last point is offset from the middle location by +stepSize
        Translation2d p1 = new Translation2d(-kStepSize, sumDCurvature2(splines));
        for (int i = 0; i < splines.size() - 1; ++i) {
            forwards(splines, controlPoints, i);
        }

        Translation2d p3 = new Translation2d(kStepSize, sumDCurvature2(splines));
        // approximate step size to minimize sumDCurvature2 along the gradient
        double stepSize = fitParabola(p1, p2, p3);

        for (int i = 0; i < splines.size() - 1; ++i) {
            finish(splines, controlPoints, stepSize, i);
        }
    }

    static void finish(
            List<HolonomicSpline> splines,
            ControlPoint[] controlPoints,
            double stepSize,
            int i) {
        Optional<Pose2d> startPose = splines.get(i).getStartPose();
        Optional<Pose2d> startPose2 = splines.get(i + 1).getStartPose();
        Optional<Pose2d> endPose = splines.get(i).getEndPose();
        Optional<Pose2d> endPose2 = splines.get(i + 1).getEndPose();
        if (startPose.isEmpty() || startPose2.isEmpty() || endPose.isEmpty() || endPose2.isEmpty()) {
            throw new IllegalArgumentException();
        }
        if (GeometryUtil.isColinear(startPose.get(), startPose2.get())
                || GeometryUtil.isColinear(endPose.get(), endPose2.get())) {
            return;
        }

        // why would this happen?
        if (controlPoints[i] == null)
            return;

        // move by the step size calculated by the parabola fit (+1 to offset for the
        // final transformation to find p3)
        controlPoints[i].ddx *= 1 + stepSize / kStepSize;
        controlPoints[i].ddy *= 1 + stepSize / kStepSize;

        splines.set(i, splines.get(i).addToSecondDerivatives(0, controlPoints[i].ddx, 0, controlPoints[i].ddy));
        splines.set(i + 1,
                splines.get(i + 1).addToSecondDerivatives(controlPoints[i].ddx, 0, controlPoints[i].ddy, 0));
    }

    static void forwards(List<HolonomicSpline> splines, ControlPoint[] controlPoints, int i) {
        Optional<Pose2d> startPose = splines.get(i).getStartPose();
        Optional<Pose2d> startPose2 = splines.get(i + 1).getStartPose();
        Optional<Pose2d> endPose = splines.get(i).getEndPose();
        Optional<Pose2d> endPose2 = splines.get(i + 1).getEndPose();
        if (startPose.isEmpty() || startPose2.isEmpty() || endPose.isEmpty() || endPose2.isEmpty()) {
            throw new IllegalArgumentException();
        }
        if (GeometryUtil.isColinear(startPose.get(), startPose2.get())
                || GeometryUtil.isColinear(endPose.get(), endPose2.get())) {
            return;
        }

        // why would this happen?
        if (controlPoints[i] == null)
            return;

        // move along the gradient by 2 times the step size amount (to return to
        // original location and move by 1 step)
        splines.set(i,
                splines.get(i).addToSecondDerivatives(0, 2 * controlPoints[i].ddx, 0, 2 * controlPoints[i].ddy));
        splines.set(i + 1, splines.get(i + 1).addToSecondDerivatives(2 * controlPoints[i].ddx, 0,
                2 * controlPoints[i].ddy, 0));
    }

    static void backwards(
            List<HolonomicSpline> splines,
            ControlPoint[] controlPoints,
            double magnitude,
            int i) {
        Optional<Pose2d> startPose = splines.get(i).getStartPose();
        Optional<Pose2d> startPose2 = splines.get(i + 1).getStartPose();
        Optional<Pose2d> endPose = splines.get(i).getEndPose();
        Optional<Pose2d> endPose2 = splines.get(i + 1).getEndPose();
        if (startPose.isEmpty() || startPose2.isEmpty() || endPose.isEmpty() || endPose2.isEmpty()) {
            throw new IllegalArgumentException();
        }
        if (GeometryUtil.isColinear(startPose.get(), startPose2.get())
                || GeometryUtil.isColinear(endPose.get(), endPose2.get())) {
            return;
        }

        // why would this happen?
        if (controlPoints[i] == null)
            return;

        // normalize to step size
        controlPoints[i].ddx *= kStepSize / magnitude;
        controlPoints[i].ddy *= kStepSize / magnitude;

        // move opposite the gradient by step size amount
        splines.set(i, splines.get(i).addToSecondDerivatives(0, -controlPoints[i].ddx, 0, -controlPoints[i].ddy));
        splines.set(i + 1,
                splines.get(i + 1).addToSecondDerivatives(-controlPoints[i].ddx, 0, -controlPoints[i].ddy, 0));
    }

    /**
     * Extract the control points from the list of splines.
     * 
     * @param splines       input splines
     * @param controlPoints output control points
     * @return sum of ddx^2+ddy^2
     */
    static double getControlPoints(
            List<HolonomicSpline> splines,
            ControlPoint[] controlPoints) {
        double magnitude = 0;
        for (int i = 0; i < splines.size() - 1; ++i) {
            // don't try to optimize colinear points
            Optional<Pose2d> startPose = splines.get(i).getStartPose();
            Optional<Pose2d> startPose2 = splines.get(i + 1).getStartPose();
            Optional<Pose2d> endPose = splines.get(i).getEndPose();
            Optional<Pose2d> endPose2 = splines.get(i + 1).getEndPose();
            if (startPose.isEmpty() || startPose2.isEmpty() || endPose.isEmpty() || endPose2.isEmpty()) {
                throw new IllegalArgumentException();
            }
            if (GeometryUtil.isColinear(startPose.get(), startPose2.get())
                    || GeometryUtil.isColinear(endPose.get(), endPose2.get())) {
                continue;
            }
            double original = sumDCurvature2(splines);

            // holds the gradient at a control point
            controlPoints[i] = new ControlPoint();

            // calculate partial derivatives of sumDCurvature2
            splines.set(i, splines.get(i).addToSecondDerivatives(0, kEpsilon, 0, 0));
            splines.set(i + 1, splines.get(i + 1).addToSecondDerivatives(kEpsilon, 0, 0, 0));
            controlPoints[i].ddx = (sumDCurvature2(splines) - original) / kEpsilon;

            splines.set(i, splines.get(i).addToSecondDerivatives(0, 0, 0, kEpsilon));
            splines.set(i + 1, splines.get(i + 1).addToSecondDerivatives(0, 0, kEpsilon, 0));
            controlPoints[i].ddy = (sumDCurvature2(splines) - original) / kEpsilon;

            // TODO: ???
            splines.set(i, splines.get(i));
            splines.set(i + 1, splines.get(i + 1));
            magnitude += controlPoints[i].ddx * controlPoints[i].ddx + controlPoints[i].ddy * controlPoints[i].ddy;
        }
        return magnitude;
    }

    /**
     * fits a parabola to 3 points
     *
     * @return the x coordinate of the vertex of the parabola
     */
    static double fitParabola(Translation2d p1, Translation2d p2, Translation2d p3) {
        double A = (p3.getX() * (p2.getY() - p1.getY()) + p2.getX() * (p1.getY() - p3.getY())
                + p1.getX() * (p3.getY() - p2.getY()));
        double B = (p3.getX() * p3.getX() * (p1.getY() - p2.getY()) + p2.getX() * p2.getX() * (p3.getY() - p1.getY())
                + p1.getX() * p1.getX() *
                        (p2.getY() - p3.getY()));
        return -B / (2 * A);
    }

}
