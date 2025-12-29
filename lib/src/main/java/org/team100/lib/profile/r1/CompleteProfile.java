package org.team100.lib.profile.r1;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.tuning.Mutable;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * A simple profile with all the things we want from a motion profile for
 * mechanisms. There are six parameters:
 * 
 * * maximum acceleration
 * * maximum deceleration (typically higher than accel, "plugging")
 * * maximum velocity
 * * "stall" acceleration for calculating back EMF
 * * maximum jerk for takeoff (optional)
 * * maximum jerk for landing (optional)
 * 
 * This works by precalculating the "goal" path on instantiation, and
 * calculating the initial path dynamically. When the initial state is close to
 * the goal path, then states are interpolated from it.
 * 
 * The initial state can be anything; the goal is stationary so it can be
 * precalculated, and this is our only real use-case anyway.
 * 
 * It's analogous to a sliding-mode controller: maximum effort to reach a
 * curve, and then moving alnog it.
 * 
 * See https://www.desmos.com/calculator/jnc7u3jg11 for useful curves.
 * 
 * See
 * https://docs.google.com/spreadsheets/d/1JdKViVSTEMZ0dRS8broub4P-f0eA6STRHHzoV0U4N5M/edit?gid=2097479642#gid=2097479642
 * for output of this model
 */
public class CompleteProfile implements IncrementalProfile {
    private static final boolean DEBUG = false;

    /** Time step for initializing the (fixed) goal path */
    private static final double DT = 0.01;
    // Extends maxV far away.
    private static final double FAR_AWAY = 1000;

    private final LoggerFactory m_log;
    private final Mutable m_maxV;
    private final Mutable m_maxAUnscaled;
    private final Mutable m_maxDUnscaled;
    private final Mutable m_stallAUnscaled;
    private final Mutable m_takeoffJ;
    private final Mutable m_landingJ;
    private final double m_scale;
    private final Mutable m_tolerance;
    final InterpolatingTreeMap<Double, Control100> m_byDistance;

    /**
     * Too-low a tolerance will produce chatter. Too-high a tolerance will produce a
     * little hiccup at the goal.
     * 
     * Jerk in the middle of the path is unlimited, because it seems complicated to
     * add a limit there.
     * 
     * @param maxV      max velocity
     * @param maxA      max acceleration (current-limited, for initial path)
     * @param maxD      max decel (higher than accel, only used for goal path)
     * @param maxStallA theoretical stall acceleration, for calculating back-EMF
     * @param takeoffJ  max jerk for takeoff, zero for unlimited
     * @param landingJ  max jerk for landing, zero for unlimited
     * @param tolerance this close to the switching curve to be on it.
     *                  also used to sense "at goal"
     */
    public CompleteProfile(
            LoggerFactory log, double maxV, double maxA, double maxD, double stallA,
            double takeoffJ, double landingJ, double tolerance) {
        if (maxV <= 0)
            throw new IllegalArgumentException("max V must be positive");
        if (maxA <= 0)
            throw new IllegalArgumentException("max A must be positive");
        if (maxD <= 0)
            throw new IllegalArgumentException("max D must be positive");
        if (stallA <= 0)
            throw new IllegalArgumentException("stall A must be positive");
        if (takeoffJ < 0)
            throw new IllegalArgumentException("takeoff J may not be negative");
        if (landingJ < 0)
            throw new IllegalArgumentException("landing J may not be positive");
        m_log = log;
        m_maxV = new Mutable(log, "maxV", maxV, this::update);
        m_maxAUnscaled = new Mutable(log, "maxA", maxA, this::update);
        m_maxDUnscaled = new Mutable(log, "maxD", maxD, this::update);
        m_stallAUnscaled = new Mutable(log, "stallA", stallA, this::update);
        m_takeoffJ = new Mutable(log, "takeoffJ", takeoffJ, this::update);
        m_landingJ = new Mutable(log, "landingJ", landingJ, this::update);
        m_scale = 1.0;
        m_tolerance = new Mutable(log, "tolerance", tolerance, this::update);
        m_byDistance = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Control100::interpolate);
        init();
    }

    public CompleteProfile(
            LoggerFactory log, Mutable maxV, Mutable maxA, Mutable maxD, Mutable stallA,
            Mutable takeoffJ, Mutable landingJ, double scale, Mutable tolerance) {
        m_log = log;
        m_maxV = maxV;
        m_maxAUnscaled = maxA;
        m_maxDUnscaled = maxD;
        m_stallAUnscaled = stallA;
        m_takeoffJ = takeoffJ;
        m_landingJ = landingJ;
        m_scale = scale;
        m_tolerance = tolerance;
        m_byDistance = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Control100::interpolate);
        init();
    }

    void update(double ignored) {
        init();
    }

    /**
     * Compute the goal path. This is done on instantiation and also anytime that
     * any of the Mutables change.
     */
    void init() {
        m_byDistance.clear();
        // This is the goal state, zero control here.
        Control100 control = new Control100();
        put(0.0, control);
        // Far-away points so that the interpolator always yields maxV.
        put(0.0, new Control100(-FAR_AWAY, m_maxV.getAsDouble(), 0));
        put(0.0, new Control100(FAR_AWAY, -m_maxV.getAsDouble(), 0));
        // control from the left, so deceleration, walking back in time
        // t is just for debugging
        double t = 0;
        for (int i = 1; i < 1000; ++i) {
            if (MathUtil.isNear(control.v(), m_maxV.getAsDouble(), m_tolerance.getAsDouble())) {
                // we're already cruising. keep cruising.
                control = new Control100(
                        control.x() - m_maxV.getAsDouble() * DT,
                        m_maxV.getAsDouble(),
                        0);
                t += DT;
                put(t, control);
            } else {
                double jerkLimitedA = jerkLimitedAccel(control);
                double nextV = control.v() - jerkLimitedA * DT;
                if (nextV > m_maxV.getAsDouble()) {
                    // maxV is achieved within DT
                    // how long does it take to get there?
                    double dt = -1.0 * (m_maxV.getAsDouble() - control.v()) / jerkLimitedA;
                    t += dt;
                    // this should be exactly at the corner.
                    control = new Control100(
                            control.x() - control.v() * dt + 0.5 * jerkLimitedA * dt * dt,
                            m_maxV.getAsDouble(),
                            jerkLimitedA);
                    put(t, control);
                    // this is zero accel, epsilon away, so that the interpolator doesn't try to
                    // match the full-accel at the corner.
                    Control100 corner = new Control100(
                            control.x() - 1e-3,
                            m_maxV.getAsDouble(),
                            0);
                    put(t, corner);
                    // the "far away" points should take care of the rest.
                    break;
                } else {
                    // Haven't reached maxV yet, keep going on the decel path.
                    control = new Control100(
                            control.x() - control.v() * DT + 0.5 * jerkLimitedA * DT * DT,
                            nextV,
                            jerkLimitedA);
                    t += DT;
                    put(t, control);
                }
            }
        }
    }

    @Override
    public Control100 calculate(double dt, Control100 setpoint, Model100 goal) {
        if (Math.abs(goal.v()) > 1e-6)
            throw new IllegalArgumentException("This profile works only with stationary goals.");

        // Negative togo => setpoint is to the left.
        final double togo = setpoint.x() - goal.x();
        if (MathUtil.isNear(0, togo, m_tolerance.getAsDouble())) {
            // Within tolerance of goal
            return goal.control();
        }

        final double maxA = accel(dt, setpoint);
        final Control100 lerp = m_byDistance.get(togo);

        // When imagining how this works, it's good to have the phase space diagram in
        // front of you. The "move right" and "move left" cases are duplicated here,
        // rather than the usual scheme of inverting the profile, in the interest of
        // clarity.

        if (togo < 0) {
            if (DEBUG)
                System.out.println("goal is to the right");
            if (setpoint.v() < 0) {
                if (DEBUG)
                    System.out.println("We're moving the wrong way (left), so brake.");
                return control(dt, setpoint, goal, togo, 1.0, getScaledD());
            }
            if (setpoint.v() + m_tolerance.getAsDouble() < lerp.v()) {
                if (DEBUG)
                    System.out.println("Setpoint is below the goal path, so push right.");
                return control(dt, setpoint, goal, togo, 1.0, maxA);
            }
            if (setpoint.v() - m_tolerance.getAsDouble() < lerp.v()) {
                if (DEBUG)
                    System.out.println("Setpoint is within tolerance of the goal path.");
                return goalPath(dt, setpoint, goal, togo, lerp.a());
            }
            if (DEBUG)
                System.out.println("Setpoint is above the goal path, so brake.");
            return control(dt, setpoint, goal, togo, -1.0, getScaledD());
        } else {
            if (DEBUG)
                System.out.println("goal is to the left");
            if (setpoint.v() > 0) {
                if (DEBUG)
                    System.out.println("We're moving the wrong way (right), so brake.");
                return control(dt, setpoint, goal, togo, -1.0, getScaledD());
            }
            if (setpoint.v() - m_tolerance.getAsDouble() > lerp.v()) {
                if (DEBUG)
                    System.out.println("Setpoint is above the goal path, so push left.");
                return control(dt, setpoint, goal, togo, -1.0, maxA);
            }
            if (setpoint.v() + m_tolerance.getAsDouble() > lerp.v()) {
                if (DEBUG)
                    System.out.println("Setpoint is within tolerance of the goal path.");
                return goalPath(dt, setpoint, goal, togo, lerp.a());
            }
            if (DEBUG)
                System.out.println("Setpoint is below the goal path, so brake.");

            return control(dt, setpoint, goal, togo, 1.0, getScaledD());
        }
    }

    @Override
    public IncrementalProfile scale(double s) {
        return new CompleteProfile(m_log,
                m_maxV, m_maxAUnscaled, m_maxDUnscaled, m_stallAUnscaled,
                m_takeoffJ, m_landingJ, s, m_tolerance);
    }

    private double getScaledA() {
        return m_scale * m_maxAUnscaled.getAsDouble();
    }

    private double getScaledD() {
        return m_scale * m_maxDUnscaled.getAsDouble();
    }

    private double getScaledStall() {
        return m_scale * m_stallAUnscaled.getAsDouble();
    }

    ///////////////////////////////////////

    private Control100 control(
            double dt,
            Control100 setpoint,
            Model100 goal,
            double togo,
            double direction,
            double a) {
        double nextX = togo + setpoint.v() * dt + direction * 0.5 * a * dt * dt;
        double nextV = setpoint.v() + direction * a * dt;
        Control100 nextLerp = m_byDistance.get(nextX);
        if (direction * nextV > direction * nextLerp.v()) {
            // The next step spans the goal path, so use the goal path.
            return new Control100(goal.x() + nextLerp.x(), nextLerp.v(), nextLerp.a());
        }
        // Setpoint is still far from the goal path, so proceed.
        return new Control100(goal.x() + nextX, nextV, direction * a);
    }

    /**
     * Get the next goal-path control near the setpoint.
     */
    private Control100 goalPath(
            double dt,
            Control100 setpoint,
            Model100 goal,
            double togo,
            double accel) {
        double nextX = togo + setpoint.v() * dt + 0.5 * accel * dt * dt;
        Control100 nextLerp = m_byDistance.get(nextX);
        return new Control100(goal.x() + nextLerp.x(), nextLerp.v(), nextLerp.a());
    }

    /**
     * Acceleration limited by the current limit, back-EMF, and the takeoff jerk
     * limit.
     * Returns a positive number.
     */
    double accel(double dt, Control100 setpoint) {
        double speedFraction = Math100.limit(Math.abs(setpoint.v()) / m_maxV.getAsDouble(), 0, 1);
        double backEmfLimit = 1 - speedFraction;
        double backEmfLimitedAcceleration = backEmfLimit * getScaledStall();
        double currentLimitedAcceleration = getScaledA();
        double jerkLimitedAcceleration = Math.abs(setpoint.a()) + m_takeoffJ.getAsDouble() * dt;

        if (DEBUG) {
            System.out.printf("fraction %5.2f backEmfLimited %5.2f currentLimited %5.2f jerklimited %5.2f\n",
                    speedFraction, backEmfLimitedAcceleration, currentLimitedAcceleration,
                    jerkLimitedAcceleration);
        }

        return Math.min(Math.min(backEmfLimitedAcceleration, currentLimitedAcceleration), jerkLimitedAcceleration);
    }

    /**
     * Put the control and its mirror on the other side of the goal
     */
    private void put(double t, Control100 c) {
        // t is just for debug
        if (DEBUG) {
            System.out.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
        }
        if (DEBUG) {
            System.out.printf("%12.4f %12.4f %12.4f %12.4f\n", t, -c.x(), -c.v(), -c.a());
        }
        m_byDistance.put(c.x(), c);
        m_byDistance.put(-c.x(), c.mult(-1.0));
    }

    /**
     * This is for the "goal path" which is always slowing down, so use the max
     * decel. The jerk limit affects the "landing".
     */
    private double jerkLimitedAccel(Control100 control) {
        if (m_landingJ.getAsDouble() < 1e-6) {
            // zero endJ means no jerk limit
            return -getScaledD();
        }
        return Math.max(-getScaledD(), control.a() - m_landingJ.getAsDouble() * DT);
    }
}
