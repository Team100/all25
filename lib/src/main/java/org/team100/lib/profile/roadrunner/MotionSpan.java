package org.team100.lib.profile.roadrunner;

/**
 * Part of a motion profile, parameterized by distance.
 * 
 * @param start state at x = 0
 * @param dx    length of this span
 */
public record MotionSpan(MotionState start, double dx) {
}
