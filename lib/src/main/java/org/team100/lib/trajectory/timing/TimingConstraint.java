package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.PathPoint;

/**
 * Timing constraints govern the assignment of a schedule to a path, creating a
 * trajectory. Different implementations focus on different aspects, e.g.
 * tippiness, wheel slip, etc. Different maneuvers may want different
 * constraints, e.g. some should be slow and precise, others fast and risky.
 * 
 * Note that this interface doesn't support jerk limiting.
 * 
 * I've gone back and forth om supporting jerk limiting, and for now I took it
 * out. It's complicated, we don't seem to need it, and mechanism slack creates
 * jerk even if the motor tries not to.
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
