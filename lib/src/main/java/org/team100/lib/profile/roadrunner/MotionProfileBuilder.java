package org.team100.lib.profile.roadrunner;

import java.util.ArrayList;
import java.util.List;

public class MotionProfileBuilder {
    private final List<MotionSegment> segments;
    private MotionState currentState;

    /**
     * Easy-to-use builder for creating motion profiles.
     *
     * @param start start motion state
     */
    public MotionProfileBuilder(MotionState start) {
        currentState = start;
        segments = new ArrayList<MotionSegment>();
    }

    /**
     * Appends a constant-jerk control for the provided duration.
     */
    MotionProfileBuilder appendJerkSegment(double jerk, double dt) {
        MotionSegment segment = new MotionSegment(
                new MotionState(currentState.getX(), currentState.getV(), currentState.getA(), jerk),
                dt);
        segments.add(segment);
        currentState = segment.end();
        return this;
    }

    /**
     * Appends a constant-acceleration control for the provided duration.
     */
    MotionProfileBuilder appendAccelerationSegment(double accel, double dt) {
        MotionSegment segment = new MotionSegment(
                new MotionState(currentState.getX(), currentState.getV(), accel, 0),
                dt);
        segments.add(segment);
        currentState = segment.end();
        return this;
    }

    /**
     * Appends a constant-velocity control for the provided duration.
     */
    MotionProfileBuilder appendVelocitySegment(double dt) {
        MotionSegment segment = new MotionSegment(
                new MotionState(currentState.getX(), currentState.getV(), 0, 0),
                dt);
        segments.add(segment);
        currentState = segment.end();
        return this;
    }

    /**
     * Appends a [MotionProfile] to the current queue of control actions.
     */
    MotionProfileBuilder appendProfile(MotionProfile profile) {
        for (MotionSegment segment : profile.getSegments()) {
            if (Math.abs(segment.start().getJ()) < 1e-6) {
                // constant acceleration
                appendAccelerationSegment(segment.start().getA(), segment.dt());
            } else {
                // constant jerk
                appendJerkSegment(segment.start().getJ(), segment.dt());
            }
        }
        return this;
    }

    /**
     * Constructs and returns the [MotionProfile] instance.
     */
    MotionProfile build() {
        return new MotionProfile(segments);
    }
}