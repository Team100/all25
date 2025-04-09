package org.team100.lib.profile.roadrunner;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.MathUtil;

/**
 * Motion profile generator with arbitrary start and end motion states and
 * dynamic constraints.
 * 
 * The main issue with this approach is the spatial sampling, which means that
 * the resulting temporal sampling depends on the profile speed. At low speed,
 * it can be quite coarse.
 */
public class DynamicProfileGenerator {

    /**
     * Generates a motion profile with dynamic maximum velocity and acceleration.
     * Uses the algorithm described in section 3.2 of
     * [Sprunk2008.pdf](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf).
     *
     * Warning:
     * Profiles may be generated incorrectly if the endpoint velocity/acceleration
     * values preclude the obedience of the motion constraints. To protect against
     * this, verify the continuity of the generated profile or keep the start and
     * goal velocities at 0.
     *
     * @param start                  start motion state
     * @param goal                   goal motion state
     * @param velocityConstraint     velocity constraint
     * @param accelerationConstraint acceleration constraint
     * @param resolution             separation between constraint samples
     */
    public static MotionProfile generateMotionProfile(
            MotionState start,
            MotionState goal,
            VelocityConstraint velocityConstraint,
            AccelerationConstraint accelerationConstraint,
            double resolution) {
        if (goal.x() < start.x()) {
            return generateMotionProfile(
                    start.flipped(),
                    goal.flipped(),
                    (s) -> velocityConstraint.get(-s),
                    (s) -> accelerationConstraint.get(-s),
                    resolution).flipped();
        }

        double length = goal.x() - start.x();
        // dx is an adjusted resolution that fits nicely within length
        // at least two samples are required to have a valid profile
        int samples = Math.max(2, (int) Math.ceil(length / resolution));

        int size = samples - 1;
        double step = length / size;

        List<MotionSpan> forwardStates = computeForward(
                start, velocityConstraint, accelerationConstraint, step, size);

        List<MotionSpan> backwardStates = computeBackward(
                goal, velocityConstraint, accelerationConstraint, step, size);

        List<MotionSpan> finalStates = merge(forwardStates, backwardStates);

        return getMotionSegments(finalStates);
    }

    /**
     * Computes the forward states.
     * 
     * Applies maximum acceleration starting at min(last velocity, max vel) on a
     * segment-by-segment basis.
     */
    static List<MotionSpan> computeForward(
            MotionState start,
            VelocityConstraint velocityConstraint,
            AccelerationConstraint accelerationConstraint,
            double step,
            int size) {
        List<MotionSpan> forwardStates = new ArrayList<>();
        MotionState lastState = start;
        for (int i = 0; i < size; ++i) {
            double displacement = start.x() + step * i;
            double maxVel = velocityConstraint.get(displacement);
            double maxAccel = accelerationConstraint.get(displacement);
            if (lastState.v() >= maxVel) {
                // the last velocity exceeds max vel so we just coast
                MotionState state = new MotionState(displacement, maxVel, 0.0, 0);
                forwardStates.add(new MotionSpan(state, step));
                lastState = evolve(state, step);
            } else {
                // compute the final velocity assuming max accel
                double finalVel = Math.sqrt(lastState.v() * lastState.v() + 2 * maxAccel * step);
                if (finalVel <= maxVel) {
                    // we're still under max vel so we're good
                    MotionState state = new MotionState(displacement, lastState.v(), maxAccel, 0);
                    forwardStates.add(new MotionSpan(state, step));
                    lastState = evolve(state, step);
                } else {
                    // we went over max vel so now we split the segment
                    double accelDx = (maxVel * maxVel - lastState.v() * lastState.v()) / (2 * maxAccel);
                    MotionState accelState = new MotionState(displacement, lastState.v(), maxAccel, 0);
                    MotionState coastState = new MotionState(displacement + accelDx, maxVel, 0.0, 0);
                    forwardStates.add(new MotionSpan(accelState, accelDx));
                    forwardStates.add(new MotionSpan(coastState, step - accelDx));
                    lastState = evolve(coastState, step - accelDx);
                }
            }
        }
        return forwardStates;
    }

    /**
     * Computes the backward states.
     * 
     * Walks backwards from the goal, computing states with maximum decel.
     */
    static List<MotionSpan> computeBackward(
            MotionState goal,
            VelocityConstraint velocityConstraint,
            AccelerationConstraint accelerationConstraint,
            double step,
            int size) {
        // linkedlist allows efficient addFirst
        LinkedList<MotionSpan> backwardStates = new LinkedList<>();
        // going back in time, so this is the "next" state not the "last" state.
        MotionState nextState = goal;
        for (int i = 0; i < size; ++i) {
            // walk backwards from the goal
            double displacement = goal.x() - step * i;
            // compute the segment constraints
            double maxVel = velocityConstraint.get(displacement);
            double maxAccel = accelerationConstraint.get(displacement);
            if (nextState.v() >= maxVel) {
                // the last velocity exceeds max vel so we just coast
                MotionState state = new MotionState(displacement, maxVel, 0.0, 0);
                // step backwards
                nextState = devolve(state, step);
                backwardStates.addFirst(new MotionSpan(nextState, step));
            } else {
                // compute the final velocity assuming max accel
                double finalVel = Math.sqrt(nextState.v() * nextState.v() + 2 * maxAccel * step);
                if (finalVel <= maxVel) {
                    // we're still under max vel so we're good
                    // note negative accel
                    MotionState state = new MotionState(displacement, nextState.v(), -maxAccel, 0);
                    nextState = devolve(state, step);
                    backwardStates.addFirst(new MotionSpan(nextState, step));
                } else {
                    // we went over max vel so now we split the segment
                    double accelDx = (maxVel * maxVel - nextState.v() * nextState.v()) / (2 * maxAccel);
                    // note negative accel
                    MotionState accelState = new MotionState(displacement, nextState.v(), -maxAccel, 0);
                    MotionState coastState = new MotionState(displacement - accelDx, maxVel, 0.0, 0);
                    backwardStates.addFirst(new MotionSpan(devolve(accelState, accelDx), accelDx));
                    nextState = devolve(coastState, step - accelDx);
                    backwardStates.addFirst(new MotionSpan(nextState, step - accelDx));
                }
            }
        }
        return backwardStates;
    }

    /** merge the forward and backward states */
    private static List<MotionSpan> merge(
            List<MotionSpan> forwardStates,
            List<MotionSpan> backwardStates) {
        List<MotionSpan> finalStates = new ArrayList<>();

        int i = 0;
        while (i < forwardStates.size() && i < backwardStates.size()) {
            // retrieve the start states and displacement deltas
            MotionState forwardStartState = forwardStates.get(i).start();
            double forwardDx = forwardStates.get(i).dx();
            MotionState backwardStartState = backwardStates.get(i).start();
            double backwardDx = backwardStates.get(i).dx();

            // if there's a discrepancy in the displacements, split the the longer chunk in
            // two and add the second
            // to the corresponding list; this guarantees that segments are always aligned
            if (!(MathUtil.isNear(forwardDx, backwardDx, 1e-6))) {
                if (forwardDx > backwardDx) {
                    // forward longer
                    forwardStates.add(
                            i + 1,
                            new MotionSpan(evolve(forwardStartState, backwardDx), forwardDx - backwardDx));
                    forwardDx = backwardDx;
                } else {
                    // backward longer
                    backwardStates.add(
                            i + 1,
                            new MotionSpan(evolve(backwardStartState, forwardDx), backwardDx - forwardDx));
                    backwardDx = forwardDx;
                }
            }

            // compute the end states (after alignment)
            MotionState forwardEndState = evolve(forwardStartState, forwardDx);
            MotionState backwardEndState = evolve(backwardStartState, backwardDx);

            if (forwardStartState.v() <= backwardStartState.v()) {
                // forward start lower
                if (forwardEndState.v() <= backwardEndState.v()) {
                    // forward end lower
                    finalStates.add(new MotionSpan(forwardStartState, forwardDx));
                } else {
                    // backward end lower
                    double intersection = intersection(
                            forwardStartState,
                            backwardStartState);
                    finalStates.add(new MotionSpan(forwardStartState, intersection));
                    finalStates.add(
                            new MotionSpan(
                                    evolve(backwardStartState, intersection),
                                    backwardDx - intersection));
                }
            } else {
                // backward start lower
                if (forwardEndState.v() >= backwardEndState.v()) {
                    // backward end lower
                    finalStates.add(new MotionSpan(backwardStartState, backwardDx));
                } else {
                    // forward end lower
                    double intersection = intersection(
                            forwardStartState,
                            backwardStartState);
                    finalStates.add(new MotionSpan(backwardStartState, intersection));
                    finalStates.add(
                            new MotionSpan(
                                    evolve(forwardStartState, intersection),
                                    forwardDx - intersection));
                }
            }
            i++;
        }
        return finalStates;
    }

    /** Turn the final states into actual time-parameterized motion segments */
    private static MotionProfile getMotionSegments(List<MotionSpan> finalStates) {
        List<MotionSegment> motionSegments = new ArrayList<MotionSegment>();
        for (MotionSpan finalState : finalStates) {
            MotionState state = finalState.start();
            double stateDx = finalState.dx();
            double dt;
            if (Math.abs(state.a()) < 1e-6) {
                dt = stateDx / state.v();
            } else {
                double discriminant = state.v() * state.v() + 2 * state.a() * stateDx;
                if (Math.abs(discriminant) < 1e-6) {
                    dt = -state.v() / state.a();
                } else {
                    dt = (Math.sqrt(discriminant) - state.v()) / state.a();
                }
            }
            motionSegments.add(new MotionSegment(state, dt));
        }

        return new MotionProfile(motionSegments);
    }

    ////////////////////////////////////////////////////////

    /** Integrates the starting state over distance (not time) dx. */
    static MotionState evolve(MotionState state, double dx) {
        double discriminant = state.v() * state.v() + 2 * state.a() * dx;
        if (discriminant < -1e-6) {
            throw new IllegalArgumentException("state " + state + " does not extend to " + dx);
        }
        if (MathUtil.isNear(discriminant, 0.0, 1e-6)) {
            return new MotionState(state.x() + dx, 0.0, state.a(), 0);
        }
        return new MotionState(state.x() + dx, Math.sqrt(discriminant), state.a(), 0);
    }

    /** Backwards in time */
    static MotionState devolve(MotionState state, double dx) {
        double discriminant = state.v() * state.v() - 2 * state.a() * dx;
        if (discriminant < -1e-6) {
            throw new IllegalArgumentException("state " + state + " does not extend to " + dx);
        }
        if (MathUtil.isNear(discriminant, 0.0, 1e-6)) {
            return new MotionState(state.x() - dx, 0.0, state.a(), 0);
        }
        return new MotionState(state.x() - dx, Math.sqrt(discriminant), state.a(), 0);
    }

    /**
     * In phase space, a constant acceleration trajectory is given by
     * 
     * v = sqrt(v0^2 + 2ax)
     * 
     * So to solve for the intersection (x,y) between two trajectories, set the two
     * velocities equal and solve for x:
     * 
     * sqrt(v1^2 + 2a1x) = sqrt(v2^2 + 2a2x)
     * v1^2-v2^2 = 2a2x - 2a1x
     * x = (v1^2-v2^2)/(2a2-2a1)
     * 
     * note that this intersection doesn't occur at the same time on each
     * trajectory.
     * 
     * see https://www.desmos.com/calculator/jnc7u3jg11
     */
    static double intersection(MotionState state1, MotionState state2) {
        return (state1.v() * state1.v() - state2.v() * state2.v())
                / (2 * state2.a() - 2 * state1.a());
    }

}
