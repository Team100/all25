package org.team100.lib.controller.simple;

import org.team100.lib.state.Model100;

/**
 * Represents a single-output feedback controller, such as PID.
 * 
 * This could have multiple inputs, e.g. like both position and velocity, but it
 * only produces one control output, which is treated here as a primitive: you
 * can apply it wherever you want (e.g. as a control effort, or a velocity
 * target for a servo, or whatever).
 */
public interface Feedback100 {

    double calculate(Model100 measurement, Model100 setpoint);

    boolean atSetpoint();

    void reset();
}
