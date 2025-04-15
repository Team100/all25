package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.reference.Setpoints1d;

/**
 * Angular position control, e.g. for swerve steering axes or arm axes.
 * 
 * Servos no longer include profiles (in order to coordinate multiple axes), so
 * this just takes setpoints, and, for onboard versions, computes feedback
 * control.
 */
public interface AngularPositionServo extends Glassy {
    /**
     * Zeros controller errors, sets setpoint to current position.
     *
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    void reset();

    /**
     * Invalidates the current profile.
     */
    public void setDutyCycle(double dutyCycle);

    void setTorqueLimit(double torqueNm);

    /**
     * Initializes the profile if necessary.
     * 
     * Sets the goal, updates the setpoint to the "next step" value towards it,
     * gives the setpoint to the outboard mechanism.
     * 
     * @param goalRad             The goal angle here wraps within [-pi, pi], using
     *                            output measurements, e.g. shaft radians, not motor
     *                            radians
     * @param feedForwardTorqueNm used for gravity or spring compensation
     */
    void setPositionProfiled(double goalRad, double feedForwardTorqueNm);

    /**
     * Invalidates the current profile, sets the setpoint directly.
     * This takes both current and next setpoints so that the implementation can
     * choose the current one for feedback and the next one for feedforward.
     */
    void setPositionDirect(Setpoints1d setpoint, double feedForwardTorqueNm);

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * @return Current position measurement, radians.
     */
    OptionalDouble getPosition();

    boolean atSetpoint();

    boolean profileDone();

    boolean atGoal();

    void stop();

    void close();

    /** for logging */
    void periodic();

}
