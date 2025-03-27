package org.team100.lib.controller.simple;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Feedback and feedforward control.
 */
public interface ProfiledController {
    /**
     * @param feedforward the desired state for the end of the next time step.
     * @param feedback    based on the error at the current instant.
     */
    public record Result(Control100 feedforward, double feedback) {
    }

    /**
     * Initializes the setpoint.
     * 
     * @param measurement current-instant measurement
     */
    void init(Model100 measurement);

    /**
     * Calculates feedforward and feedback.
     * 
     * Feedback is based on the error between the current-instant measurement and
     * the setpoint calculated in the past, which was also intended for the current
     * instant.
     * 
     * Feedforward is based on the profile at the end of the next time step.
     * 
     * This remembers the feedforward and uses it on the next step.
     * 
     * @param measurement current-instant measurement
     * @param goal        final desired state
     */
    Result calculate(Model100 measurement, Model100 goal);

    /**
     * The profile is at the goal. The measurement may not be. This is useful for
     * chaining actions without waiting for the measurement, e.g. to transition from
     * "profiled" motion to "hold position" motion using the same goal.
     */
    boolean profileDone();

    Model100 getSetpoint();

    /**
     * The feedback controller error is within its tolerance, i.e. we are following
     * the profile well.
     */
    boolean atSetpoint();

    /**
     * The profile has reached the goal and the feedback error is within tolerance,
     * i.e. our path is complete.
     * 
     * This doesn't use current measurements, it uses whatever inputs we saw in
     * calculate() most recently.
     */
    boolean atGoal(Model100 goal);

    /**
     * to return resources used within, this is really for tests.
     */
    void close();

}