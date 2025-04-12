package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;

/**
 * Linear position control, e.g. for elevators.
 */
public interface LinearPositionServo extends Glassy {
    /**
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    void reset();

     /**
     * This is movement and force on the output.
     * 
     * @param goalM             meters
     * @param feedForwardTorque used for gravity compensation
     */
    void setPosition(double goalM, double feedForwardTorqueNm);

    OptionalDouble getPosition();

    OptionalDouble getVelocity();

    boolean profileDone();

    void stop();

    void close();

    void periodic();
}
