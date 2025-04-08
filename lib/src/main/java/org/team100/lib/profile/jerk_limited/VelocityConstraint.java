package org.team100.lib.profile.jerk_limited;


/**
 * Motion profile velocity constraint.
 */
@FunctionalInterface
public interface VelocityConstraint {
    /**
     * Returns the maximum profile velocity at displacement [s].
     */
    double get(double s);
}