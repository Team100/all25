package org.team100.lib.profile.roadrunner;

/**
 * Kinematic state of a motion profile at any given time.
 */
public record MotionState(double x, double v, double a, double j) {

    /**
     * Returns the [MotionState] at time [t].
     */
    public MotionState get(double t) {
        return new MotionState(
                x + v * t + a / 2 * t * t + j / 6 * t * t * t,
                v + a * t + j / 2 * t * t,
                a + j * t,
                j);
    }

    /**
     * Returns a flipped (negated) version of the state.
     */
    public MotionState flipped() {
        return new MotionState(-x, -v, -a, -j);
    }

    /**
     * Returns the state with velocity, acceleration, and jerk zeroed.
     */
    public MotionState stationary() {
        return new MotionState(x, 0.0, 0.0, 0.0);
    }

    public String toString() {
        return String.format("(x=%.3f, v=%.3f, a=%.3f, j=%.3f)", x, v, a, j);
    }
}