package org.team100.lib.profile.roadrunner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.MathUtil;

/**
 * Motion profile generator with arbitrary start and end motion states and
 * dynamic constraints.
 */
public class DynamicProfileGenerator {

    static class EvaluatedConstraint {
        double maxVel;
        double maxAccel;

        public EvaluatedConstraint(double maxVel, double maxAccel) {
            this.maxVel = maxVel;
            this.maxAccel = maxAccel;
        }

    }

    /**
     * Generates a motion profile with dynamic maximum velocity and acceleration.
     * Uses the algorithm described in
     * section 3.2 of
     * [Sprunk2008.pdf](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf).
     * Warning:
     * Profiles may be generated incorrectly if the endpoint velocity/acceleration
     * values preclude the obedience of the
     * motion constraints. To protect against this, verify the continuity of the
     * generated profile or keep the start and
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
                    new VelocityConstraint() {
                        public double get(double s) {
                            return velocityConstraint.get(-s);
                        }
                    },
                    new AccelerationConstraint() {
                        public double get(double s) {
                            return accelerationConstraint.get(-s);
                        }
                    },
                    resolution).flipped();
        }

        double length = goal.x() - start.x();
        // dx is an adjusted resolution that fits nicely within length
        // at least two samples are required to have a valid profile
        int samples = Math.max(2, (int) Math.ceil(length / resolution));

        List<EvaluatedConstraint> constraintsList = new ArrayList<>();
        double x = start.x();
        double step = length / (samples - 1);
        int size = samples - 1;
        for (int i = 0; i < size; ++i) {
            x += step;
            constraintsList.add(new EvaluatedConstraint(velocityConstraint.get(x), accelerationConstraint.get(x)));
        }

        List<MotionSpan> forwardStates = computeForward(
                start, velocityConstraint, accelerationConstraint, step, size);

        List<MotionSpan> backwardStates = computeBackward(
                goal, velocityConstraint, accelerationConstraint, constraintsList, step, size);

        List<MotionSpan> finalStates = merge(forwardStates, backwardStates);

        return getMotionSegments(finalStates);
    }

    /**
     * Computes the forward states.
     * 
     * Applies maximum acceleration starting at min(last velocity, max vel) on a
     * segment-by-segment basis.
     * 
     * @return a list of spans.
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


    /** compute the backward states */
    static List<MotionSpan> computeBackward(
            MotionState goal,
            VelocityConstraint velocityConstraint,
            AccelerationConstraint accelerationConstraint,
            List<EvaluatedConstraint> constraintsList,
            double step,
            int size) {

        List<MotionSpan> backwardPass = backwardPass(
                new MotionState(0.0, goal.v(), goal.a(), 0),
                size,
                step,
                constraintsList,
                velocityConstraint,
                accelerationConstraint);

        List<MotionSpan> backwardStates = new ArrayList<>();
        for (MotionSpan it : backwardPass) {
            MotionState motionState = it.start();
            double dx = it.dx();
            MotionState evolved = evolve(motionState, dx);
            backwardStates.add(new MotionSpan(
                    new MotionState(
                            goal.x() - evolved.x(),
                            evolved.v(),
                            -evolved.a(),
                            0),
                    dx));
        }

        Collections.reverse(backwardStates);
        return backwardStates;
    }

    static List<MotionSpan> backwardPass(
            MotionState goal,
            int size,
            double dx,
            List<EvaluatedConstraint> constraintsList,
            VelocityConstraint vConstraint,
            AccelerationConstraint aConstraint) {
        List<EvaluatedConstraint> constraints = new ArrayList<EvaluatedConstraint>(constraintsList);
        Collections.reverse(constraints);

        List<MotionSpan> backwardStates = new ArrayList<>();

        MotionState lastState = goal;
        for (int i = 0; i < size; ++i) {
            double displacement = goal.x() + dx * i;
            EvaluatedConstraint constraint = constraints.get(i);
            // compute the segment constraints
            double maxVel = constraint.maxVel;
            double maxAccel = constraint.maxAccel;

            // double maxVel = vConstraint.get(displacement);
            // double maxAccel = aConstraint.get(displacement);

            if (lastState.v() >= maxVel) {
                // the last velocity exceeds max vel so we just coast
                MotionState state = new MotionState(displacement, maxVel, 0.0, 0);
                backwardStates.add(new MotionSpan(state, dx));
                lastState = evolve(state, dx);
            } else {
                // compute the final velocity assuming max accel
                double finalVel = Math.sqrt(lastState.v() * lastState.v() + 2 * maxAccel * dx);
                if (finalVel <= maxVel) {
                    // we're still under max vel so we're good
                    MotionState state = new MotionState(displacement, lastState.v(), maxAccel, 0);
                    backwardStates.add(new MotionSpan(state, dx));
                    lastState = evolve(state, dx);
                } else {
                    // we went over max vel so now we split the segment
                    double accelDx = (maxVel * maxVel - lastState.v() * lastState.v()) / (2 * maxAccel);
                    MotionState accelState = new MotionState(displacement, lastState.v(), maxAccel, 0);
                    MotionState coastState = new MotionState(displacement + accelDx, maxVel, 0.0, 0);
                    backwardStates.add(new MotionSpan(accelState, accelDx));
                    backwardStates.add(new MotionSpan(coastState, dx - accelDx));
                    lastState = evolve(coastState, dx - accelDx);
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
