package org.team100.lib.controller.simple;

import org.team100.lib.profile.Profile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * A profiled controller combines a source of references (e.g. a trapezoid, or
 * any other method) and a source of feedback (e.g. PID, or any other method).
 */
public class ProfiledController implements Controller100 {

    public ProfiledController(Profile100 profile, Controller100 controller) {

    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        // use modulus instead
    }

    public Control100 calculate(Model100 measurement, Model100 setpoint) {
        return null;
    }

}
