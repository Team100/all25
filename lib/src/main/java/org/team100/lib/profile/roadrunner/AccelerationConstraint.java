package org.team100.lib.profile.roadrunner;

/**
 * Motion profile acceleration constraint.
 */
@FunctionalInterface
public interface AccelerationConstraint {
    /**
     * Returns the maximum profile acceleration at displacement [s].
     */
    double get(double s);
}