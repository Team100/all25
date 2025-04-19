package org.team100.lib.profile.roadrunner;


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