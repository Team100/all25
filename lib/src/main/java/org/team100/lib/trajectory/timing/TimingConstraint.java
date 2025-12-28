package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.PathPoint;

/**
 * Timing constraints govern the assignment of a schedule to a path, creating a
 * trajectory. Different implementations focus on different aspects, e.g.
 * tippiness, wheel slip, etc. Different maneuvers may want different
 * constraints, e.g. some should be slow and precise, others fast and risky.
 * 
 * Note that this interface doesn't support jerk limiting in a simple way.
 * If you want to limit jerk at the start and/or end of a trajectory,
 * you can implement that using the acceleration and deceleration limits,
 * with a constraint that is aware of the endpoint locations.
 */
public interface TimingConstraint {
    /**
     * Maximum allowed pathwise velocity, m/s.
     * 
     * Always positive.
     */
    double maxV(PathPoint state);

    /**
     * Maximum allowed pathwise acceleration, m/s^2.
     * 
     * Always positive.
     */
    double maxAccel(PathPoint state, double velocityM_S);

    /**
     * Maximum allowed pathwise deceleration, m/s^2.
     * 
     * Always negative.
     */
    double maxDecel(PathPoint state, double velocityM_S);
}
