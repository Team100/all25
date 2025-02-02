package org.team100.lib.controller.simple;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public interface Controller100 {
    /**
     * Produce the desired state for the next time step.
     */
    Control100 calculate(Model100 measurement, Model100 setpoint);

    /**
     * If the most-recent calculation inputs were within the tolerance.
     */
    boolean atSetpoint();

    void reset();
}
