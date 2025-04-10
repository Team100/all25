package org.team100.lib.profile.timed;

import org.team100.lib.profile.IncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * A simple profile to implement all the things we want from a
 * motion profile for mechanisms:
 * 
 * * current limit
 * * back-EMF limit
 * * jerk-limited tapering
 * 
 * So there are five parameters:
 * 
 * * maximum acceleration
 * * maximum deceleration (typically higher than accel, "plugging")
 * * maximum velocity
 * * "stall" acceleration for calculating back EMF
 * * maximum jerk, for tapering
 * 
 * the initial state can be anything.
 * 
 * the goal is always stationary.
 * 
 * This works by precalculating the "goal" path on instantiation, and
 * calculating the initial path dynamically. when the initial state is close to
 * the goal path, then states are interpolated from it.
 * 
 * so this is actually an incremental profile.
 * 
 * Note that only stationary goal states are allowed, so that the goal path can
 * be precalculated (and because 100% of our actual paths have stationary
 * goals).
 * 
 * It's similar to a sliding-mode controller: maximum effort to reach a
 * manifold, and then moving alnog it.
 * 
 * see https://www.desmos.com/calculator/jnc7u3jg11 for useful curves.
 */
public class CompleteProfile implements IncrementalProfile {
    private static final boolean DEBUG = false;
    private static final double DT = 0.02;
    // private static final double FAR_AWAY = 1000;

    private final double m_maxV;
    private final double m_maxA;
    private final double m_maxD;
    private final double m_stallA;
    private final double m_maxJ;
    private final double m_tolerance;
    final InterpolatingTreeMap<Double, Control100> m_byDistance;

    /**
     * 
     * @param maxV      max velocity
     * @param maxA      max acceleration (current-limited, constant)
     * @param maxD      max decel (usually higher than accel, only used for goal path)
     * @param maxStallA theoretical stall acceleration, for calculating back-EMF
     * @param maxJ      max jerk (only used for goal path tapering)
     * @param tolerance this close to the switching surface to be on it
     */
    public CompleteProfile(double maxV, double maxA, double maxD, double stallA, double maxJ, double tolerance) {
        m_maxV = maxV;
        m_maxA = maxA;
        m_maxD = maxD;
        m_stallA = stallA;
        m_maxJ = maxJ;
        m_tolerance = tolerance;
        InverseInterpolator<Double> keyInterpolator = InverseInterpolator.forDouble();
        Interpolator<Control100> valueInterpolator = Control100::interpolate;
        m_byDistance = new InterpolatingTreeMap<>(keyInterpolator, valueInterpolator);
        // this is the goal state, zero control here.
        Control100 c = new Control100();
        m_byDistance.put(0.0, c);
        if (DEBUG)
            Util.printf("%12.4f %12.4f %12.4f %12.4f\n", 0.0, c.x(), c.v(), c.a());
        // control from the left, so deceleration, walking back in time
        // these are decel profiles so use maxD.
        double a = -maxD;
        double t = 0;
        for (int i = 1; i < 1000; ++i) {
            if (MathUtil.isNear(c.v(), maxV, tolerance)) {
                // we're already cruising. keep cruising.
                c = new Control100(
                        c.x() - maxV * DT,
                        maxV,
                        0);
                m_byDistance.put(c.x(), c);
                t += DT;
                if (DEBUG)
                    Util.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
            } else {
                double aa = Math.max(a, c.a() - maxJ * DT);
                double v = c.v() - aa * DT;
                if (v > maxV) {
                    // maxV is achieved within DT
                    // how long does it take to get there?
                    double dt = -1.0 * (maxV - c.v()) / aa;
                    // this should be exactly at the corner.
                    c = new Control100(
                            c.x() - c.v() * dt + 0.5 * aa * dt * dt,
                            maxV,
                            aa);
                    m_byDistance.put(c.x(), c);
                    m_byDistance.put(c.x() - 1e-6, new Control100(
                            c.x() - c.v() * dt + 0.5 * aa * dt * dt,
                            maxV,
                            0));
                    t += dt;
                    if (DEBUG)
                        Util.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
                } else {
                    c = new Control100(
                            c.x() - c.v() * DT + 0.5 * aa * DT * DT,
                            v,
                            aa);
                    m_byDistance.put(c.x(), c);
                    t += DT;
                    if (DEBUG)
                        Util.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
                }
            }
        }
        // control from the right, walking back in time
        a = maxD;
        t = 0;
        c = new Control100();
        for (int i = 1; i < 1000; ++i) {
            if (MathUtil.isNear(c.v(), -maxV, tolerance)) {
                // we're already cruising. keep cruising.
                c = new Control100(
                        c.x() + maxV * DT,
                        -maxV,
                        0);
                m_byDistance.put(c.x(), c);
                t += DT;
                if (DEBUG)
                    Util.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
            } else {
                double aa = Math.min(a, c.a() + maxJ * DT);
                double v = c.v() - aa * DT;
                if (v < -maxV) {
                    // maxV is achieved within DT
                    // how long does it take to ge tthere?
                    double dt = -1.0 * (-maxV - c.v()) / aa;
                    // this should be exactly at the corner.
                    c = new Control100(
                            c.x() - c.v() * dt + 0.5 * a * dt * dt,
                            -maxV,
                            aa);
                    m_byDistance.put(c.x(), c);
                    m_byDistance.put(c.x() + 1e-6,
                            new Control100(
                                    c.x() - c.v() * dt + 0.5 * aa * dt * dt,
                                    -maxV,
                                    0));
                    t += dt;
                    if (DEBUG)
                        Util.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
                } else {
                    c = new Control100(
                            c.x() - c.v() * DT + 0.5 * aa * DT * DT,
                            v,
                            aa);
                    m_byDistance.put(c.x(), c);
                    t += DT;
                    if (DEBUG)
                        Util.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
                }
            }
        }
    }

    @Override
    public Control100 calculate(double dt, Model100 setpoint, Model100 goal) {
        if (Math.abs(goal.v()) > 1e-6)
            throw new IllegalArgumentException("This profile works only with stationary goals.");
        if (DEBUG)
            Util.printf("setpoint %s goal %s\n", setpoint, goal);

        // negative number = setpoint is to the left.
        double togo = setpoint.x() - goal.x();
        if (MathUtil.isNear(0, togo, m_tolerance)) {
            if (DEBUG)
                Util.printf("at goal, togo %s\n", togo);
            return goal.control();
        }
        double a = accel(setpoint.v());
        Control100 lerp = m_byDistance.get(togo);
        if (DEBUG)
            Util.printf("togo %f lerp %s\n", togo, lerp);
        if (togo < 0) {
            if (DEBUG)
                Util.println("goal is to the right");
            if (setpoint.v() + m_tolerance < lerp.v()) {
                // setpoint is below the goal path, go faster to get there.
                if (DEBUG)
                    Util.println("setpoint velocity is less positive than goal path: speed up");
                // would the next point be on the other side?
                double v = setpoint.v() + a * dt;
                if (DEBUG)
                    Util.printf("switch? v %f lerp v%f lerp a %f a %f\n", v, lerp.v(), lerp.a() * dt, a);
                if (v > lerp.v() + lerp.a() * dt) {
                    if (DEBUG)
                        Util.println("switch");
                    // the next step spans the switching curve, so just take the lerp
                    // (this is a bit conservative compared to the intersection but it's simpler)
                    return new Control100(
                            goal.x() + lerp.x() + lerp.v() * dt + 0.5 * lerp.a() * dt * dt,
                            lerp.v() + lerp.a() * dt,
                            lerp.a());
                }
                return new Control100(
                        setpoint.x() + setpoint.v() * dt + 0.5 * a * dt * dt,
                        v,
                        a);
            } else if (setpoint.v() - m_tolerance < lerp.v()) {
                if (DEBUG)
                    Util.printf("setpoint is within tolerance of lerp %s\n", lerp);
                double lv = lerp.v() * dt;
                if (DEBUG)
                    Util.printf("lv %f\n", lv);
                Control100 r = new Control100(
                        goal.x() + lerp.x() + lv + 0.5 * lerp.a() * dt * dt,
                        lerp.v() + lerp.a() * dt,
                        lerp.a());
                if (DEBUG)
                    Util.printf("r %s\n", r);
                return r;
            } else {
                // setpoint is above the goal path, slow down to get there.
                if (DEBUG)
                    Util.println("setpoint velocity is more positive than goal path: slow down");
                // would the next point be on the other side?

                double v = setpoint.v() - m_maxD * dt;
                if (v < lerp.v()) {
                    return new Control100(
                            goal.x() + lerp.x() + lerp.v() * dt + 0.5 * lerp.a() * dt * dt,
                            lerp.v() + lerp.a() * dt,
                            lerp.a());
                }
                return new Control100(
                        setpoint.x() + setpoint.v() * dt - 0.5 * m_maxD * dt * dt,
                        v,
                        -m_maxD);
            }
        } else {
            if (DEBUG)
                Util.println("goal is to the left: use negative velocity");
            if (setpoint.v() - m_tolerance > lerp.v()) {
                // setpoint is above the goal path, go faster (to the left) to get there.
                if (DEBUG)
                    Util.println("setpoint velocity is less negative than goal path: speed up");
                return new Control100(
                        setpoint.x() + setpoint.v() * dt - 0.5 * a * dt * dt,
                        setpoint.v() - a * dt,
                        -a);
            } else if (setpoint.v() + m_tolerance > lerp.v()) {
                if (DEBUG)
                    Util.println("setpoint is within tolerance of lerp");
                return new Control100(
                        goal.x() + lerp.x() + lerp.v() * dt + 0.5 * lerp.a() * dt * dt,
                        lerp.v() + lerp.a() * dt,
                        lerp.a());
            } else {
                // setpoint is below the goal path, slow down (i.e. push to the right) to get
                // there.
                if (DEBUG)
                    Util.println("setpoint velocity is more negative than goal path: slow down");
                return new Control100(
                        setpoint.x() + setpoint.v() * dt + 0.5 * m_maxD * dt * dt,
                        setpoint.v() + m_maxD * dt,
                        m_maxD);
            }
        }
    }

    double accel(double velocity) {
        double speedFraction = Math100.limit(velocity / m_maxV, 0, 1);
        double backEmfLimit = 1 - speedFraction;
        double backEmfLimitedAcceleration = backEmfLimit * m_stallA;
        double currentLimitedAcceleration = m_maxA;
        if (DEBUG) {
            Util.printf("speedFraction %5.2f backEmfLimitedAcceleration %5.2f currentLimitedAcceleration %5.2f\n",
                    speedFraction, backEmfLimitedAcceleration, currentLimitedAcceleration);
        }
        return Math.min(backEmfLimitedAcceleration, currentLimitedAcceleration);
    }

}
