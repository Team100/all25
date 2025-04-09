package org.team100.lib.profile.timed;

import org.team100.lib.profile.IncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
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
 */
public class CompleteProfile implements IncrementalProfile {
    private static final boolean DEBUG = true;
    private static final double DT = 0.02;

    private final double m_maxV;
    private final double m_maxA;
    private final double m_tolerance;
    final InterpolatingTreeMap<Double, Control100> m_byDistance;

    public CompleteProfile(double maxV, double maxA, double tolerance) {
        m_maxV = maxV;
        m_maxA = maxA;
        m_tolerance = tolerance;
        InverseInterpolator<Double> keyInterpolator = InverseInterpolator.forDouble();
        Interpolator<Control100> valueInterpolator = Control100::interpolate;
        m_byDistance = new InterpolatingTreeMap<>(keyInterpolator, valueInterpolator);
        // this is the goal state, zero control here.
        Control100 c = new Control100();
        m_byDistance.put(0.0, c);
        // control from the left, walking back in time
        double a = -maxA;
        for (int i = 0; i < 100; ++i) {
            c = new Control100(
                    c.x() - c.v() * DT - 0.5 * a * DT * DT,
                    c.v() - a * DT,
                    a);
            m_byDistance.put(c.x(), c);
        }
        // control from the right, walking back in time
        a = maxA;
        c = new Control100();
        for (int i = 0; i < 100; ++i) {
            c = new Control100(
                    c.x() - c.v() * DT - 0.5 * a * DT * DT,
                    c.v() - a * DT,
                    a);
            m_byDistance.put(c.x(), c);
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
                return new Control100(
                        setpoint.x() + setpoint.v() * dt + 0.5 * m_maxA * dt * dt,
                        setpoint.v() + m_maxA * dt,
                        m_maxA);
            } else if (setpoint.v() - m_tolerance < lerp.v()) {
                if (DEBUG)
                    Util.println("setpoint is within tolerance of lerp");
                return new Control100(
                        goal.x() + lerp.x() + lerp.v() * dt + 0.5 * lerp.a() * dt * dt,
                        lerp.v() + lerp.a() * dt,
                        lerp.a());
            } else {
                // setpoint is above the goal path, slow down to get there.
                if (DEBUG)
                    Util.println("setpoint velocity is more positive than goal path: slow down");
                return new Control100(
                        setpoint.x() + setpoint.v() * dt - 0.5 * m_maxA * dt * dt,
                        setpoint.v() - m_maxA * dt,
                        -m_maxA);
            }
        } else {
            if (DEBUG)
                Util.println("goal is to the left: use negative velocity");
            if (setpoint.v() - m_tolerance > lerp.v()) {
                // setpoint is above the goal path, go faster (to the left) to get there.
                if (DEBUG)
                    Util.println("setpoint velocity is less negative than goal path: speed up");
                return new Control100(
                        setpoint.x() + setpoint.v() * dt - 0.5 * m_maxA * dt * dt,
                        setpoint.v() - m_maxA * dt,
                        -m_maxA);
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
                        setpoint.x() + setpoint.v() * dt + 0.5 * m_maxA * dt * dt,
                        setpoint.v() + m_maxA * dt,
                        m_maxA);
            }
        }
    }

}
