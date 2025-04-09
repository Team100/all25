package org.team100.lib.profile.roadrunner;

/** Part of a motion profile, parameterized by distance. */
public record MotionSpan(MotionState start, double dx) {
}
