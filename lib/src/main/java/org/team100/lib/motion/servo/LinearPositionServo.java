package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.reference.Setpoints1d;

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
     * Initializes the profile if necessary.
     * This is movement and force on the output.
     * 
     * @param goalM             meters
     * @param feedForwardTorque used for gravity compensation
     */
    void setPositionProfiled(double goalM, double feedForwardTorqueNm);

    /**
     * Invalidates the current profile, sets the setpoint directly.
     * This takes both current and next setpoints so that the implementation can
     * choose the current one for feedback and the next one for feedforward.
     */
    void setPositionDirect(Setpoints1d setpoint, double feedForwardTorqueNm);

    OptionalDouble getPosition();

    OptionalDouble getVelocity();

    boolean profileDone();

    boolean atGoal(double xTolerance, double vTolerance);

    void stop();

    void close();

    void periodic();
}
