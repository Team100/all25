package org.team100.lib.motion.servo;

import java.util.OptionalDouble;
import org.team100.lib.dashboard.Glassy;

/**
 * Represents a servo whose output is measured in linear units -- this is
 * usually relevant for wheeled mechanisms, where the surface speed of the wheel
 * is the important thing. For example, a conveyor belt drive is a spinning
 * thing, but the important thing is the linear movement of the belt.
 * 
 * This is part of the project to remove generics from the motion components; I
 * think generics make things too hard to read.
 */
public interface LinearVelocityServo extends Glassy {
    void reset();

    /**
     * Set velocity and compute implied acceleration based on the previous call,
     * using TimedRobot100.LOOP_PERIOD_S.  If you call this more often, you'll
     * get weird results.
     * 
     * @param setpointM_S desired speed, m/s
     */
    void setVelocityM_S(double setpointM_S);

    /**
     * @param setpoint   desired speed, m/s
     * @param setpoint_2 desired acceleration m/s^2
     */
    void setVelocity(double setpoint, double setpoint_2);

    /** meters/sec. Note this can be noisy, maybe filter it. */
    OptionalDouble getVelocity();

    void stop();

    OptionalDouble getDistance();

    /** For logging */
    void periodic();

}
