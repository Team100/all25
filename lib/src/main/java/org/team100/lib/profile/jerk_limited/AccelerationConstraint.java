package org.team100.lib.profile.jerk_limited;

/**
 * Motion profile acceleration constraint.
 */
public interface AccelerationConstraint {
    /**
     * Returns the maximum profile acceleration at displacement [s].
     */
    double get(double s);
}