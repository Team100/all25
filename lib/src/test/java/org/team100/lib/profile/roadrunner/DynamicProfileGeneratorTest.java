package org.team100.lib.profile.roadrunner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;
import org.team100.lib.profile.roadrunner.DynamicProfileGenerator.EvaluatedConstraint;

public class DynamicProfileGeneratorTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.001;

    @Test
    void testGenerateMotionProfile() {
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(5, 0, 0, 0);
        VelocityConstraint v = new VelocityConstraint() {

            @Override
            public double get(double s) {
                return 1;
            }

        };
        AccelerationConstraint a = new AccelerationConstraint() {

            @Override
            public double get(double s) {
                return 1;
            }

        };
        double resolution = 1;
        MotionProfile p = DynamicProfileGenerator.generateMotionProfile(start, goal, v, a, resolution);

        assertEquals(7, p.getSegments().size());
        assertEquals(0, p.get(0).getX(), kDelta);
        assertEquals(0.5, p.get(1).getX(), kDelta);
        assertEquals(1.5, p.get(2).getX(), kDelta);
        assertEquals(2.5, p.get(3).getX(), kDelta);
        assertEquals(3.5, p.get(4).getX(), kDelta);
        assertEquals(4.5, p.get(5).getX(), kDelta);
        assertEquals(5.0, p.get(6).getX(), kDelta);

        assertEquals(6.0, p.duration(), kDelta);

        MotionState s0 = p.get(0);
        assertEquals(0, s0.getV(), kDelta);

        MotionProfile p1 = p.append(p);
        assertEquals(12, p1.duration(), kDelta);

        MotionState s1 = p.get(1);
        assertEquals(1, s1.getV(), kDelta);
    }

    @Test
    void testEvolve() {
        {
            // this is a bit nonsensical, this state would never move.
            MotionState s0 = new MotionState(0, 0, 0, 0);
            MotionState s1 = DynamicProfileGenerator.evolve(s0, 1);
            assertEquals(1, s1.getX(), kDelta);
            assertEquals(0, s1.getV(), kDelta);
            assertEquals(0, s1.getA(), kDelta);
            assertEquals(0, s1.getJ(), kDelta);
        }
        {
            // nonzero v, zero a, end v is the same
            MotionState s0 = new MotionState(0, 1, 0, 0);
            MotionState s1 = DynamicProfileGenerator.evolve(s0, 1);
            assertEquals(1, s1.getX(), kDelta);
            assertEquals(1, s1.getV(), kDelta);
            assertEquals(0, s1.getA(), kDelta);
            assertEquals(0, s1.getJ(), kDelta);
        }
        {
            // nonzero a, end v is more
            MotionState s0 = new MotionState(0, 0, 1, 0);
            MotionState s1 = DynamicProfileGenerator.evolve(s0, 1);
            assertEquals(1, s1.getX(), kDelta);
            assertEquals(1.414, s1.getV(), kDelta);
            assertEquals(1, s1.getA(), kDelta);
            assertEquals(0, s1.getJ(), kDelta);
        }
        {
            // v is low, a is negative
            MotionState s0 = new MotionState(0, 0.1, -10, 0);
            assertThrows(IllegalArgumentException.class, ()-> DynamicProfileGenerator.evolve(s0, 1));
        }
    }

    @Test
    void testForwardPass() {
        MotionState start = new MotionState(1, 0, 0, 0);
        int size = 3;
        double step = 0.3;
        List<EvaluatedConstraint> constraints = List.of(
                new EvaluatedConstraint(2, 6),
                new EvaluatedConstraint(2, 6),
                new EvaluatedConstraint(2, 6));
        // forward pass just accelerates forever, like the initial part of the
        // trapezoid.
        // note the relocation to zero
        List<MotionSpan> states = DynamicProfileGenerator.forwardPass(
                new MotionState(0.0, start.getV(), start.getA(), 0), size, step, constraints, (s) -> 2.0, (s) -> 6.0);
        assertEquals(4, states.size());
        verify(states.get(0), 0, 0, 6, 0, 0.3); // full-length segment at max A
        verify(states.get(1), 0.3, 1.897, 6, 0, 0.033); // short segment to max V
        verify(states.get(2), 0.333, 2, 0, 0, 0.267);
        verify(states.get(3), 0.6, 2, 0, 0, 0.3);

        // this is the transmogrification that the current code does.
        // it seems to just shift the position.
        List<MotionSpan> forwardStates = states.stream().map((it) -> {
            MotionState motionState = it.start();
            double dx = it.dx();
            return new MotionSpan(new MotionState(
                    motionState.getX() + start.getX(),
                    motionState.getV(),
                    motionState.getA(),
                    0), dx);
        }).collect(Collectors.toList());
        assertEquals(4, forwardStates.size());
        // position shifted to start
        verify(forwardStates.get(0), 1, 0, 6, 0, 0.3); // full-length segment at max A
        verify(forwardStates.get(1), 1.3, 1.897, 6, 0, 0.033); // short segment to max V
        verify(forwardStates.get(2), 1.333, 2, 0, 0, 0.267);
        verify(forwardStates.get(3), 1.6, 2, 0, 0, 0.3);
    }

    @Test
    void testComputeForward() {
        MotionState start = new MotionState(1, 0, 0, 0);
        int size = 3;
        double step = 0.3;
        List<EvaluatedConstraint> constraints = List.of(
                new EvaluatedConstraint(2, 6),
                new EvaluatedConstraint(2, 6),
                new EvaluatedConstraint(2, 6));
        List<MotionSpan> forwardStates = DynamicProfileGenerator.computeForward(
                start, (s) -> 2.0, (s) -> 6.0, constraints, step, size);
        assertEquals(4, forwardStates.size());
        // position shifted to start
        verify(forwardStates.get(0), 1, 0, 6, 0, 0.3); // full-length segment at max A
        verify(forwardStates.get(1), 1.3, 1.897, 6, 0, 0.033); // short segment to max V
        verify(forwardStates.get(2), 1.333, 2, 0, 0, 0.267);
        verify(forwardStates.get(3), 1.6, 2, 0, 0, 0.3);
    }

    @Test
    void testBackwardsPass() {
        // the current code reuses the forward pass code and reverses the result, which
        // is confusing.
        MotionState goal = new MotionState(1.9, 0, 0, 0);
        int size = 3;
        double step = 0.3;
        List<EvaluatedConstraint> constraints = List.of(
                new EvaluatedConstraint(2, 6),
                new EvaluatedConstraint(2, 6),
                new EvaluatedConstraint(2, 6));
        // forward pass just accelerates forever, like the initial part of the
        // trapezoid.
        // note the relocation to zero
        List<MotionSpan> states = DynamicProfileGenerator.forwardPass(
                new MotionState(0.0, goal.getV(), goal.getA(), 0),
                size, step, constraints, (s) -> 2.0, (s) -> 6.0);
        assertEquals(4, states.size());
        verify(states.get(0), 0, 0, 6, 0, 0.3); // full-length segment at max A
        verify(states.get(1), 0.3, 1.897, 6, 0, 0.033); // short segment to max V
        verify(states.get(2), 0.333, 2, 0, 0, 0.267);
        verify(states.get(3), 0.6, 2, 0, 0, 0.3);

        // this replaces the initial state of each pair with the end state
        List<MotionSpan> states2 = states.stream().map((it) -> {
            MotionState motionState = it.start();
            double dx = it.dx();
            return new MotionSpan(DynamicProfileGenerator.evolve(motionState, dx), dx);
        }).collect(Collectors.toList());
        assertEquals(4, states2.size());
        verify(states2.get(0), 0.3, 1.897, 6, 0, 0.3); // full-length segment at max A
        verify(states2.get(1), 0.333, 2, 6, 0, 0.033); // short segment to max V
        verify(states2.get(2), 0.6, 2, 0, 0, 0.267);
        verify(states2.get(3), 0.9, 2, 0, 0, 0.3);

        // this is the transmogrification that the current code does.
        // it seems to just shift the position.
        List<MotionSpan> backwardStates = states2.stream().map((it) -> {
            MotionState motionState = it.start();
            double dx = it.dx();
            return new MotionSpan(new MotionState(
                    goal.getX() - motionState.getX(),
                    motionState.getV(),
                    -motionState.getA(),
                    0), dx);
        }).collect(Collectors.toList());

        Collections.reverse(backwardStates);

        assertEquals(4, backwardStates.size());
        // position shifted to goal and reversed, so this
        // shows max effort from the goal towards the start.
        verify(backwardStates.get(0), 1, 2, 0, 0, 0.3);
        verify(backwardStates.get(1), 1.3, 2, 0, 0, 0.267);
        verify(backwardStates.get(2), 1.567, 2, -6, 0, 0.033);
        verify(backwardStates.get(3), 1.6, 1.897, -6, 0, 0.3);
    }

    @Test
    void testComputeBackwards() {
        MotionState goal = new MotionState(1.9, 0, 0, 0);
        int size = 3;
        double step = 0.3;
        List<EvaluatedConstraint> constraints = List.of(
                new EvaluatedConstraint(2, 6),
                new EvaluatedConstraint(2, 6),
                new EvaluatedConstraint(2, 6));
        List<MotionSpan> backwardStates = DynamicProfileGenerator.computeBackward(
                goal, (s) -> 2.0, (s) -> 6.0, constraints, step, size);
        assertEquals(4, backwardStates.size());
        verify(backwardStates.get(0), 1, 2, 0, 0, 0.3);
        verify(backwardStates.get(1), 1.3, 2, 0, 0, 0.267);
        verify(backwardStates.get(2), 1.567, 2, -6, 0, 0.033);
        verify(backwardStates.get(3), 1.6, 1.897, -6, 0, 0.3);
    }

    @Test
    void testIntersection() {
        MotionState s0 = new MotionState(0, 0, 1, 0);
        MotionState s1 = new MotionState(0, 0.5, 0.5, 0);
        double c = DynamicProfileGenerator.intersection(s0, s1);
        assertEquals(0.25, c, kDelta);
    }

    void verify(MotionSpan p, double x, double v, double a, double j, double dx) {
        assertEquals(x, p.start().getX(), kDelta);
        assertEquals(v, p.start().getV(), kDelta);
        assertEquals(a, p.start().getA(), kDelta);
        assertEquals(j, p.start().getJ(), kDelta);
        assertEquals(dx, p.dx(), kDelta);

    }

}
