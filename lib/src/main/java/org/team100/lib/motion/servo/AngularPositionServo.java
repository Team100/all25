package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.reference.Setpoints1d;

/**
 * Angular position control, e.g. for swerve steering axes or arm axes.
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

    public void setDutyCycle(double dutyCycle);

    void setTorqueLimit(double torqueNm);

    /**
     * TODO: remove this.
     * 
     * Sets the goal, updates the setpoint to the "next step" value towards it,
     * gives the setpoint to the outboard mechanism.
     * 
     * @param goalRad             The goal angle here wraps within [-pi, pi], using
     *                            output measurements, e.g. shaft radians, not motor
     *                            radians
     * @param feedForwardTorqueNm used for gravity or spring compensation
     */
    void setPositionGoal(double goalRad, double feedForwardTorqueNm);

    /** No profile here. */
    void setPositionSetpoint(Setpoints1d setpoint, double feedForwardTorqueNm);

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
