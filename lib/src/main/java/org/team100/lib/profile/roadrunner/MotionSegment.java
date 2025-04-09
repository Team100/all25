package org.team100.lib.profile.roadrunner;

/**
 * Time-parameterized segment of a profile with constant acceleration
 * 
 * @param start state at t = 0
 * @param dt    duration of this segment
 */
public record MotionSegment(MotionState start, double dt) {

    /**
     * @param t in range [0, duration]
     */
    MotionState get(double t) {
        return start.get(t);
    }

    /**
     * State at the end of the segment.
     */
    MotionState end() {
        return start.get(dt);
    }

    /**
     * Returns a reversed version of the segment. Note: it isn't possible to reverse
     * a segment completely so this
     * method only guarantees that the start and end velocities will be swapped.
     */
    MotionSegment reversed() {
        MotionState end = end();
        MotionState state = new MotionState(end.x(), end.v(), -end.a(), end.j());
        return new MotionSegment(state, dt);
    }

    /**
     * Returns a flipped (negated) version of the segment.
     */
    MotionSegment flipped() {
        return new MotionSegment(start.flipped(), dt);
    }

    public String toString() {
        return String.format("(%s, %f)", start, dt);
    }
}