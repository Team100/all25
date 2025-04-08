package org.team100.lib.profile.roadrunner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;

/**
 * Motion profile generator with arbitrary start and end motion states and
 * dynamic constraints.
 */
public class DynamicProfileGenerator {

    private static class EvaluatedConstraint {
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
        if (goal.getX() < start.getX()) {
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

        double length = goal.getX() - start.getX();
        // dx is an adjusted resolution that fits nicely within length
        // at least two samples are required to have a valid profile
        int samples = Math.max(2, (int) Math.ceil(length / resolution));

        DoubleProgression s = DoubleProgression.fromClosedInterval(0.0, length, samples);
        List<EvaluatedConstraint> constraintsList = StreamSupport.stream(s.plus(start.getX()).spliterator(), false)
                .map((it) -> new EvaluatedConstraint(
                        velocityConstraint.get(it),
                        accelerationConstraint.get(it)))
                .collect(Collectors.toList());

        // compute the forward states
        List<Pair<MotionState, Double>> forwardStates = forwardPass(
                new MotionState(0.0, start.getV(), start.getA(), 0),
                s,
                constraintsList).stream().map((it) -> {
                    MotionState motionState = it.getFirst();
                    double dx = it.getSecond();
                    return new Pair<>(new MotionState(
                            motionState.getX() + start.getX(),
                            motionState.getV(),
                            motionState.getA(),
                            0), dx);
                }).collect(Collectors.toList());

        // compute the backward states
        List<EvaluatedConstraint> backwardsConstraints = new ArrayList<EvaluatedConstraint>(constraintsList);
        Collections.reverse(backwardsConstraints);
        List<Pair<MotionState, Double>> backwardStates = forwardPass(
                new MotionState(0.0, goal.getV(), goal.getA(), 0),
                s,
                backwardsConstraints).stream().map((it) -> {
                    MotionState motionState = it.getFirst();
                    double dx = it.getSecond();
                    return new Pair<>(afterDisplacement(motionState, dx), dx);
                }).map((it) -> {
                    MotionState motionState = it.getFirst();
                    double dx = it.getSecond();
                    return new Pair<>(
                            new MotionState(
                                    goal.getX() - motionState.getX(),
                                    motionState.getV(),
                                    -motionState.getA(),
                                    0),
                            dx);
                }).collect(Collectors.toList());
        Collections.reverse(backwardStates);

        // merge the forward and backward states
        List<Pair<MotionState, Double>> finalStates = new ArrayList<Pair<MotionState, Double>>();

        int i = 0;
        while (i < forwardStates.size() && i < backwardStates.size()) {
            // retrieve the start states and displacement deltas
            MotionState forwardStartState = forwardStates.get(i).getFirst();
            double forwardDx = forwardStates.get(i).getSecond();
            MotionState backwardStartState = backwardStates.get(i).getFirst();
            double backwardDx = backwardStates.get(i).getSecond();

            // if there's a discrepancy in the displacements, split the the longer chunk in
            // two and add the second
            // to the corresponding list; this guarantees that segments are always aligned
            if (!(MathUtil.isNear(forwardDx, backwardDx, 1e-6))) {
                if (forwardDx > backwardDx) {
                    // forward longer
                    forwardStates.add(
                            i + 1,
                            new Pair<>(afterDisplacement(forwardStartState, backwardDx), forwardDx - backwardDx));
                    forwardDx = backwardDx;
                } else {
                    // backward longer
                    backwardStates.add(
                            i + 1,
                            new Pair<>(afterDisplacement(backwardStartState, forwardDx), backwardDx - forwardDx));
                    backwardDx = forwardDx;
                }
            }

            // compute the end states (after alignment)
            MotionState forwardEndState = afterDisplacement(forwardStartState, forwardDx);
            MotionState backwardEndState = afterDisplacement(backwardStartState, backwardDx);

            if (forwardStartState.getV() <= backwardStartState.getV()) {
                // forward start lower
                if (forwardEndState.getV() <= backwardEndState.getV()) {
                    // forward end lower
                    finalStates.add(new Pair<>(forwardStartState, forwardDx));
                } else {
                    // backward end lower
                    double intersection = intersection(
                            forwardStartState,
                            backwardStartState);
                    finalStates.add(new Pair<>(forwardStartState, intersection));
                    finalStates.add(
                            new Pair<>(
                                    afterDisplacement(backwardStartState, intersection),
                                    backwardDx - intersection));
                }
            } else {
                // backward start lower
                if (forwardEndState.getV() >= backwardEndState.getV()) {
                    // backward end lower
                    finalStates.add(new Pair<>(backwardStartState, backwardDx));
                } else {
                    // forward end lower
                    double intersection = intersection(
                            forwardStartState,
                            backwardStartState);
                    finalStates.add(new Pair<>(backwardStartState, intersection));
                    finalStates.add(
                            new Pair<>(
                                    afterDisplacement(forwardStartState, intersection),
                                    forwardDx - intersection));
                }
            }
            i++;
        }

        // turn the final states into actual time-parameterized motion segments
        List<MotionSegment> motionSegments = new ArrayList<MotionSegment>();
        for (Pair<MotionState, Double> finalState : finalStates) {
            MotionState state = finalState.getFirst();
            double stateDx = finalState.getSecond();
            double dt;
            if (Math.abs(state.getA()) < 1e-6) {
                dt = stateDx / state.getV();
            } else {
                double discriminant = state.getV() * state.getV() + 2 * state.getA() * stateDx;
                if (Math.abs(discriminant) < 1e-6) {
                    dt = -state.getV() / state.getA();
                } else {
                    dt = (Math.sqrt(discriminant) - state.getV()) / state.getA();
                }
            }
            motionSegments.add(new MotionSegment(state, dt));
        }

        return new MotionProfile(motionSegments);
    }

    ////////////////////////////////////////////////////////

    // execute a forward pass that consists of applying maximum acceleration
    // starting at min(last velocity, max vel)
    // on a segment-by-segment basis
    private static List<Pair<MotionState, Double>> forwardPass(
            MotionState start,
            DoubleProgression displacements,
            List<EvaluatedConstraint> constraints) {
        List<Pair<MotionState, Double>> forwardStates = new ArrayList<Pair<MotionState, Double>>();

        double dx = displacements.getStep();

        MotionState lastState = start;
        int lengths = Math.min(displacements.size(), constraints.size());
        for (int i = 0; i < lengths; ++i) {
            double displacement = displacements.get(i);
            EvaluatedConstraint constraint = constraints.get(i);
            // compute the segment constraints
            double maxVel = constraint.maxVel;
            double maxAccel = constraint.maxAccel;

            if (lastState.getV() >= maxVel) {
                // the last velocity exceeds max vel so we just coast
                MotionState state = new MotionState(displacement, maxVel, 0.0, 0);
                forwardStates.add(new Pair<>(state, dx));
                lastState = afterDisplacement(state, dx);
            } else {
                // compute the final velocity assuming max accel
                double finalVel = Math.sqrt(lastState.getV() * lastState.getV() + 2 * maxAccel * dx);
                if (finalVel <= maxVel) {
                    // we're still under max vel so we're good
                    MotionState state = new MotionState(displacement, lastState.getV(), maxAccel, 0);
                    forwardStates.add(new Pair<>(state, dx));
                    lastState = afterDisplacement(state, dx);
                } else {
                    // we went over max vel so now we split the segment
                    double accelDx = (maxVel * maxVel - lastState.getV() * lastState.getV()) / (2 * maxAccel);
                    MotionState accelState = new MotionState(displacement, lastState.getV(), maxAccel, 0);
                    MotionState coastState = new MotionState(displacement + accelDx, maxVel, 0.0, 0);
                    forwardStates.add(new Pair<>(accelState, accelDx));
                    forwardStates.add(new Pair<>(coastState, dx - accelDx));
                    lastState = afterDisplacement(coastState, dx - accelDx);
                }
            }
        }

        return forwardStates;
    }

    private static MotionState afterDisplacement(MotionState state, double dx) {
        double discriminant = state.getV() * state.getV() + 2 * state.getA() * dx;
        if (MathUtil.isNear(discriminant, 0.0, 1e-6)) {
            return new MotionState(state.getX() + dx, 0.0, state.getA(), 0);
        }
        return new MotionState(state.getX() + dx, Math.sqrt(discriminant), state.getA(), 0);
    }

    private static double intersection(MotionState state1, MotionState state2) {
        return (state1.getV() * state1.getV() - state2.getV() * state2.getV())
                / (2 * state2.getA() - 2 * state1.getA());
    }

}
