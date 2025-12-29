package org.team100.lib.profile.r1;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.tuning.Mutable;

import edu.wpi.first.math.MathUtil;

/**
 * This uses the approach from LaValle 2023: between any two points in phase
 * space, the optimal acceleration-limited path is via two parabolas, perhaps
 * with a velocity limit.
 * 
 * The notation here is from that paper. Every point in phase space is
 * intersected with two minimum-time parabolas: one corresponding to maximum
 * acceleration, the other to maximum deceleration. The paper was concerned with
 * coordinating multiple axes, so there's a lot in it about non-minimum-time
 * paths. For our purpose here, the simple minimum-time case is all that
 * matters.
 * 
 * The simplest case links a state with zero velocity to another state with
 * position to the right, and also with zero velocity: the result is a
 * symmetrical pair of parabolas, an initial one for acceleration ("I" for
 * "initial") and the second for deceleration to the goal ("G" for "goal"). The
 * intersection is called the "switching point." A path with initial
 * acceleration followed by deceleration to the goal is called "I+G-". The other
 * pairing is also possible, for example the goal could be to the left of the
 * initial state, so the initial acceleration to get there would be negative.
 * 
 * The same picture applies to nonzero-velocity cases, and to cases with
 * velocities of opposite sign: any two states can be linked by either an I+G-
 * or I-G+ path.
 * 
 * Adding a velocity limit means that the "switching point" may be clipped off,
 * so the path is something like I+CG-, "C" for "cruise."
 * 
 * So there are three kinds of boundaries: I-G, I-cruise, and cruise-G.
 * 
 * https://arxiv.org/pdf/2210.01744.pdf
 * 
 * This class is different from the WPI profile class -- it keeps no state, it
 * just takes one dt step at a time. It doesn't have the notion of a profile
 * being "completed" -- if you want that, compare the output to the goal. The
 * main benefit of the approach here is that it correctly handles cases
 * involving moving end states that the WPI code does not handle correctly.
 * 
 * It might be slower around the switching points, since it can call itself once
 * or twice, once per segment.
 * 
 * Oct 3 2024, a new requirement is to compute the profile duration, for the
 * purpose of coordinating multiple profiles. In general, this sort of
 * coordination is complicated, involving three different "switching point"
 * possibilities, but for the goal-at-rest case, there's just one ("x_switch"
 * from the paper) and i think you can just scale the constraints. For an
 * implementation of the general case, see team100/studies2023/rrt, specifically
 * the RRTStar classes, which include coordination in their path solvers.
 * 
 * note: both the wpi and 100 profiles fail to produce useful feedforward when
 * the distance is reachable in one time step, i.e. high accel and velocity
 * limits.
 * 
 * Nov 4 2025: constraints are Mutables.
 */
public class TrapezoidIncrementalProfile implements IncrementalProfile {
    private static final boolean DEBUG = false;

    private final LoggerFactory m_log;
    private final Mutable m_maxVelocity;
    private final Mutable m_maxAccelerationUnscaled;
    private final double m_scale;
    private final Mutable m_tolerance;

    /**
     * Note the logger name here selects the Mutable values, so be sure it's unique,
     * if you want unique values.
     */
    public TrapezoidIncrementalProfile(
            LoggerFactory log, double maxVel, double maxAccel, double tolerance) {
        m_log = log;
        m_scale = 1;
        m_maxVelocity = new Mutable(log, "maxVel", maxVel);
        m_maxAccelerationUnscaled = new Mutable(log, "maxAccel", maxAccel);
        m_tolerance = new Mutable(log, "tolerance", tolerance);
    }

    public TrapezoidIncrementalProfile(
            LoggerFactory log, Mutable maxVel, Mutable maxAccel, double scale, Mutable tolerance) {
        m_log = log;
        m_maxVelocity = maxVel;
        m_maxAccelerationUnscaled = maxAccel;
        m_scale = scale;
        m_tolerance = tolerance;
    }

    @Override
    public TrapezoidIncrementalProfile scale(double s) {
        return new TrapezoidIncrementalProfile(
                m_log,
                m_maxVelocity,
                m_maxAccelerationUnscaled,
                s,
                m_tolerance);
    }

    private double getScaledAccel() {
        return m_scale * m_maxAccelerationUnscaled.getAsDouble();
    }

    /**
     * Return the control for dt in the future.
     * 
     * Note order of the arguments: initial state first, then goal.
     * 
     * Input velocities are clamped to the velocity constraint.
     * 
     * Input accelerations are ignored: jerk is unmanaged.
     * 
     * Output acceleration is the *profile* acceleration at dt, it is not
     * necessarily the same as the actuation required to reach the output state from
     * the initial state. The only difference occurs when the dt period spans a
     * boundary between accel and decel, or accel/decel and cruise. In those cases,
     * the reported acceleration will be from the other side of the boundary.
     * 
     * Therefore, if you blindly use the reported acceleration as the only input to
     * a real system, it will tend to get ahead of the profile, but only by one time
     * period.
     * 
     * Returns the goal if the intial is within tolerance of it.
     */
    @Override
    public Control100 calculate(double dt, final Control100 initialRaw, final Model100 goalRaw) {
        if (DEBUG) {
            System.out.printf("calculateWithETA %5.3f %s %s\n", dt, initialRaw, goalRaw);
        }
        // Too-high initial speed is handled with braking
        if (initialRaw.v() > m_maxVelocity.getAsDouble()) {
            if (DEBUG) {
                System.out.printf("positive entry speed too fast, braking %s\n", initialRaw.toString());
            }
            return full(dt, initialRaw, -1);
        }
        if (initialRaw.v() < -m_maxVelocity.getAsDouble()) {
            if (DEBUG) {
                System.out.printf("negative entry speed too fast, braking %s\n", initialRaw.toString());
            }
            return full(dt, initialRaw, 1);
        }
        Control100 initial = initialRaw;
        // Too-high goal speed is not allowed
        if (goalRaw.v() > m_maxVelocity.getAsDouble() || goalRaw.v() < -m_maxVelocity.getAsDouble()) {
            System.out.println("WARNING: Goal velocity is higher than profile velocity");
        }
        Model100 goal = limitVelocity(goalRaw);

        if (goal.control().near(initial, m_tolerance.getAsDouble())) {
            if (DEBUG) {
                System.out.print("at goal\n");
            }
            return goal.control();
        }

        // Calculate the ETA to each switch point, or NaN if there's no valid path.
        double t1IplusGminus = t1IplusGminus(initial, goal);
        double t1IminusGplus = t1IminusGplus(initial, goal);

        if (Double.isNaN(t1IminusGplus) && Double.isNaN(t1IplusGminus)) {
            System.out.println("WARNING: Both I-G+ and I+G- are NaN, this should never happen");
            return initial;
        }

        if (Double.isNaN(t1IplusGminus)) {
            // t1IminusGplus is ok
            // the valid path is I-G+, assume we're on I-
            if (DEBUG) {
                System.out.print("assume we're on I-\n");
            }
            return handleIminus(dt, initial, goal, t1IminusGplus);
        }

        if (Double.isNaN(t1IminusGplus)) {
            // t1IplusGminus is ok
            // the valid path is I+G-, assume we're on I+
            if (DEBUG) {
                System.out.print("assume we're on I+\n");
            }
            return handleIplus(dt, initial, goal, t1IplusGminus);
        }

        // There can be one path with zero duration, indicating that we're on the goal
        // path at the switch point. In that case, we want to switch immediately and
        // proceed to the goal.
        dt = truncateDt(dt, initial, goal);
        if (MathUtil.isNear(0, t1IminusGplus, 1e-12)) {
            if (DEBUG) {
                System.out.print("assume we're on G+\n");
            }
            return full(dt, initial, 1);
        }
        if (MathUtil.isNear(0, t1IplusGminus, 1e-12)) {
            if (DEBUG) {
                System.out.print("assume we're on G-\n");
            }
            return full(dt, initial, -1);
        }

        // There can be two non-zero-duration paths. As above, this happens when we're
        // on the goal path. The difference is that in this case, the goal has non-zero
        // velocity. One path goes directly to the goal, but it's also possible to make
        // a little loop in phase space, backing up and ending up in the same place, on
        // the way to the goal. We want to avoid these little loops.
        if (t1IminusGplus > t1IplusGminus) {
            if (DEBUG) {
                System.out.print("we're on G+\n");
            }
            return full(dt, initial, 1);
        }
        if (DEBUG) {
            System.out.print("we're on G-\n");
        }
        return full(dt, initial, -1);
    }

    /** Clamp state velocity to the profile limit. */
    private Model100 limitVelocity(final Model100 s) {
        return new Model100(
                s.x(),
                MathUtil.clamp(s.v(), -m_maxVelocity.getAsDouble(), m_maxVelocity.getAsDouble()));
    }

    private Control100 handleIplus(double dt, Control100 initial, Model100 goal, double timeToSwitch) {
        if (MathUtil.isNear(timeToSwitch, 0, 1e-12)) {
            // switch eta is zero: go to the goal via G-
            if (DEBUG) {
                System.out.printf("timeToSwitch %f, go to G-\n", timeToSwitch);
            }
            return full(truncateDt(dt, initial, goal), initial, -1);
        }
        // how much time to get to cruise? (remember initial v is positive)
        double timeToCruise = (m_maxVelocity.getAsDouble() - initial.v()) / getScaledAccel();

        if (timeToSwitch < dt && timeToSwitch < timeToCruise) {
            // We Encounter G- during dt, before cruise, so switch.
            if (DEBUG) {
                System.out.print("switch is soon\n");
            }
            return traverseSwitch(dt, initial, goal, timeToSwitch, 1);
        }

        if (timeToCruise < dt) {
            // we encounter vmax during dt.
            // what's the switching position?
            double x = initial.x()
                    + initial.v() * timeToCruise
                    + 0.5 * getScaledAccel() * Math.pow(timeToCruise, 2);
            Control100 nextState = new Control100(x, m_maxVelocity.getAsDouble());
            return keepCruising(dt - timeToCruise, nextState, goal);
        }
        // We will not encounter any boundary during dt
        Control100 result = full(dt, initial, 1);

        // but we still need to know the full duration.
        //
        // the two possibilities are I+G- and I+C+G-
        if (timeToSwitch < timeToCruise) {
            // we hit the switching point first
            // so if we proceed for t1, what velocity will we be at?
            if (DEBUG) {
                System.out.print("no cruise\n");
            }
            return result;
        }
        // we hit cruise first
        if (DEBUG) {
            System.out.print("go cruise\n");
        }
        return result;
    }

    /**
     * t1 is the time to the I-G+ switch point, which might be beyond the velocity
     * constraint.
     */
    private Control100 handleIminus(double dt, Control100 initial, Model100 goal, double timeToSwitch) {

        if (MathUtil.isNear(timeToSwitch, 0, 1e-12)) {
            // Switch ETA is zero: go to the goal via G+
            return full(truncateDt(dt, initial, goal), initial, 1);
        }
        // how much time to get to cruise? (remember initial v is negative)
        double timeToCruise = (m_maxVelocity.getAsDouble() + initial.v()) / getScaledAccel();

        if (timeToSwitch < dt && timeToSwitch < timeToCruise) {
            // We encounter G+ during dt, so switch.
            return traverseSwitch(dt, initial, goal, timeToSwitch, -1);
        }
        if (timeToCruise < dt) {
            // we encounter vmax during dt.
            // what's the switching position?
            double x = initial.x()
                    + initial.v() * timeToCruise
                    - 0.5 * getScaledAccel() * Math.pow(timeToCruise, 2);
            Control100 nextState = new Control100(x, -m_maxVelocity.getAsDouble());
            return keepCruisingMinus(dt - timeToCruise, nextState, goal);
        }
        // We will not encounter any boundary during dt, so the resulting state is just
        // "full throttle for dt"
        Control100 result = full(dt, initial, -1);

        // but we still need to know the full duration.
        //
        // the two possibilities are I-G+ and I-C-G+
        if (timeToSwitch < timeToCruise) {
            // we hit the switching point first
            return result;
        }
        // we hit cruise first
        return result;
    }

    /** At positive cruising speed, keep going. */
    Control100 keepCruising(double dt, Control100 initial, Model100 goal) {
        if (DEBUG) {
            System.out.printf("keep cruising %s\n", initial);
        }

        // We're already at positive cruising speed, which means G- is next.
        // will we reach it during dt?
        // this is the x location of the v0 intercept of the goal path
        double c_minus = c_minus(goal.control());
        if (DEBUG) {
            System.out.printf("c_minus %6.3f\n", c_minus);
        }
        // the G- value at current velocity (vmax)
        double gminus = c_minus - Math.pow(m_maxVelocity.getAsDouble(), 2) / (2 * getScaledAccel());
        // distance to go to the G- intersection
        double dc = gminus - initial.x();
        // time to go to the G- intersection
        double durationToGMinus = dc / m_maxVelocity.getAsDouble();
        if (MathUtil.isNear(0, durationToGMinus, 1e-12)) {
            // we are at the intersection of vmax and G-, so head down G-
            return full(truncateDt(dt, initial, goal), initial, -1);
        }
        if (durationToGMinus < dt) {
            // we reach G- before the end of dt, so we spend part of the time
            // getting there, and the remaining time going down G-. we want
            // to return the end state on G-
            double tremaining = dt - durationToGMinus;
            if (DEBUG) {
                System.out.printf("tremaining %6.3f\n", tremaining);
            }
            return calculate(tremaining, new Control100(gminus, m_maxVelocity.getAsDouble()), goal);
        }
        // we won't reach G-, so cruise for all of dt.
        return new Control100(
                initial.x() + m_maxVelocity.getAsDouble() * dt,
                m_maxVelocity.getAsDouble(),
                0);
    }

    Control100 keepCruisingMinus(double dt, Control100 initial, Model100 goal) {
        // We're already at negative cruising speed, which means G+ is next.
        // will we reach it during dt?
        double c_plus = c_plus(goal.control());
        double gplus = c_plus + Math.pow(m_maxVelocity.getAsDouble(), 2) / (2 * getScaledAccel());
        // negative
        double dc = gplus - initial.x();
        // time to go to the G+ intersection
        double durationToGPlus = dc / -m_maxVelocity.getAsDouble();
        if (MathUtil.isNear(0, durationToGPlus, 1e-12)) {
            // We're at the intersection of -vmax and G+, so head up G+
            return full(truncateDt(dt, initial, goal), initial, 1);
        }
        if (durationToGPlus < dt) {
            double tremaining = dt - durationToGPlus;
            return calculate(tremaining, new Control100(gplus, -m_maxVelocity.getAsDouble()), goal);
        }
        // we won't reach G+, so cruise for all of dt
        return new Control100(
                initial.x() - m_maxVelocity.getAsDouble() * dt,
                -m_maxVelocity.getAsDouble(),
                0);
    }

    /**
     * Travel to the switching point, and then the remainder of time on the goal
     * path.
     */
    private Control100 traverseSwitch(double dt, Control100 in_initial, final Model100 goal, double t1,
            double direction) {
        // t1 is the time before the switch
        // t2 is the time after
        double t2 = dt - t1;
        if (DEBUG) {
            System.out.printf("traverse switch %5.3f %5.3f\n", t1, t2);
        }
        // if the remaining time is ~zero, then just go full, we'll catch the switch the
        // next time.
        if (t2 < 1e-6) {
            if (DEBUG) {
                System.out.print("switch is at endpoint, go full.\n");
            }
            Control100 result = full(dt, in_initial, direction);
            return result;
        }

        // first get to the switching point
        double x = in_initial.x() + in_initial.v() * t1
                + 0.5 * direction * getScaledAccel() * Math.pow(t1, 2);
        double v = in_initial.v() + direction * getScaledAccel() * t1;

        // just use the same method for the second part
        // note this is slower than the code below so maybe put it back
        Control100 s = new Control100(x, v);
        if (DEBUG) {
            System.out.printf("switch state %s\n", s);
        }

        return calculate(t2, s, goal);
    }

    /** Returns a shorter dt to avoid overshooting the goal state. */
    private double truncateDt(double dt, Control100 in_initial, Model100 in_goal) {
        double dtg = durationAtMaxA(in_initial.v(), in_goal.v());
        return Math.min(dt, dtg);
    }

    /**
     * duration of the maximum-acceleration path from initial velocity to goal
     * velocity. this is the duration of the *actual* path from initial to goal, if
     * they lie on the same constant-acceleration parabola.
     */
    private double durationAtMaxA(double v_i, double v_g) {
        return Math.abs((v_i - v_g) / getScaledAccel());
    }

    /**
     * Extrapolate from the initial state at full acceleration for the duration dt.
     * This function is unaware of the goal per se; call it when you know what
     * direction to go and when you're sure you can proceed for the whole dt time
     * period.
     */
    private Control100 full(double dt, Control100 in_initial, double direction) {
        if (DEBUG) {
            System.out.printf("full x_i %s\n", in_initial);
        }
        double x = in_initial.x() + in_initial.v() * dt
                + 0.5 * direction * getScaledAccel() * Math.pow(dt, 2);
        double v = in_initial.v() + direction * getScaledAccel() * dt;
        double a = direction * getScaledAccel();
        return new Control100(x, v, a);
    }

    /**
     * Time to switch point for I+G- path, or NaN if there is no path.
     * 
     * Note this ignores the velocity constraint.
     */
    double t1IplusGminus(Control100 initial, Model100 goal) {
        double q_dot_switch = qDotSwitchIplusGminus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.v(), q_dot_switch, 1e-6))
            return 0;
        double t1 = (q_dot_switch - initial.v()) / getScaledAccel();
        if (t1 < 0) {
            return Double.NaN;
        }
        return t1;
    }

    /**
     * Time to switch point for I-G+ path, or NaN if there is no path.
     * 
     * Note this ignores the velocity constraint.
     */
    double t1IminusGplus(Control100 initial, Model100 goal) {
        double q_dot_switch = qDotSwitchIminusGplus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.v(), q_dot_switch, 1e-6))
            return 0;

        double t1 = (q_dot_switch - initial.v()) / (-1.0 * getScaledAccel());
        if (t1 < 0) {
            return Double.NaN;
        }
        return t1;
    }

    /**
     * Velocity of I+ at the switching point of the "switch" path.
     * 
     * "switch" path using I+G- means the goal has to be to the right of the "s"
     * shaped curve including I.
     */
    double qDotSwitchIplusGminus(Control100 initial, Model100 goal) {
        if (initial.equals(goal.control()))
            return initial.v();

        // intercept of I-
        double c_minus = c_minus(initial);
        // intercept of I+
        double c_plus = c_plus(initial);
        // position of I- at the velocity of goal
        double p_minus = c_minus - Math.pow(goal.v(), 2) / (2 * getScaledAccel());
        double p_plus = c_plus + Math.pow(goal.v(), 2) / (2 * getScaledAccel());

        // "limit" path we don't want.
        if (goal.v() <= initial.v() && goal.x() < p_minus)
            return Double.NaN;
        if (goal.v() > initial.v() && goal.x() < p_plus)
            return Double.NaN;

        // progress along I+
        double d = qSwitchIplusGminus(initial, goal) - c_plus(initial);
        // prevent rounding errors
        if (d < 0)
            d = 0;
        return Math.sqrt(2 * getScaledAccel() * d);
    }

    /**
     * Velocity of G+ at the switching point of the "switch" path.
     * 
     * "switch" path using I-G+ means the goal has to be to the left of the I- curve
     * for goal.v less than i.v, and to the left of the I+ curve for goal.v > i.v
     */
    double qDotSwitchIminusGplus(Control100 initial, Model100 goal) {
        if (initial.equals(goal.control()))
            return goal.v();

        // intercept of I-
        double c_minus = c_minus(initial);
        // intercept of I+
        double c_plus = c_plus(initial);
        // position of I- at the velocity of goal
        double p_minus = c_minus - Math.pow(goal.v(), 2) / (2 * getScaledAccel());
        double p_plus = c_plus + Math.pow(goal.v(), 2) / (2 * getScaledAccel());

        // "limit" path we don't want.

        if (goal.v() <= initial.v() && goal.x() > p_minus)
            return Double.NaN;
        if (goal.v() > initial.v() && goal.x() > p_plus)
            return Double.NaN;

        // progress along I-
        double d = qSwitchIminusGplus(initial, goal) - c_plus(goal.control());
        // prevent rounding errors
        if (d < 0)
            d = 0;

        return -1.0 * Math.sqrt(2 * getScaledAccel() * d);
    }

    /**
     * Position at the midpoint between the intercepts of the positive-acceleration
     * path through the initial state and the negative-acceleration path through the
     * goal state, i.e. the I+G- path.
     */
    double qSwitchIplusGminus(Control100 initial, Model100 goal) {
        return (c_plus(initial) + c_minus(goal.control())) / 2;
    }

    /**
     * Midpoint position for the I-G+ path.
     */
    double qSwitchIminusGplus(Control100 initial, Model100 goal) {
        return (c_minus(initial) + c_plus(goal.control())) / 2;
    }

    /** Intercept of negative-acceleration path intersecting s */
    double c_minus(Control100 s) {
        return s.x() - Math.pow(s.v(), 2) / (-2.0 * getScaledAccel());
    }

    /** Intercept of positive-acceleration path intersecting s */
    double c_plus(Control100 s) {
        return s.x() - Math.pow(s.v(), 2) / (2.0 * getScaledAccel());
    }

    // for testing
    double t1(Control100 initial, Model100 goal) {
        double t1IplusGminus = t1IplusGminus(initial, goal);
        double t1IminusGplus = t1IminusGplus(initial, goal);
        if (Double.isNaN(t1IplusGminus)) {
            return t1IminusGplus;
        }
        if (Double.isNaN(t1IminusGplus)) {
            return t1IplusGminus;
        }
        // this only happens when the positions are the same and the velocities are
        // opposite, i.e. they're on the same trajectory, in which case we want to
        // switch immediately
        return 0;
    }

    public double getMaxVelocity() {
        return m_maxVelocity.getAsDouble();
    }

    public double getMaxAcceleration() {
        return getScaledAccel();
    }

    public double getTolerance() {
        return m_tolerance.getAsDouble();
    }
}
