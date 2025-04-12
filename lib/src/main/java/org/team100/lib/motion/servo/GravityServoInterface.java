package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.state.Control100;

// this is for refactoring the gravity servo
public interface GravityServoInterface extends Glassy {

    /** Zeros controller errors, sets setpoint to current position. */
    void reset();

    OptionalDouble getPositionRad();

    /** set position with zero velocity */
    default void setPosition(double goalRad) {
        setState(new Control100(goalRad, 0));
    }

    /**
     * allow moving end-state
     * TODO: disallow moving end-state, we never use it.
     */
    void setState(Control100 goal);

    void stop();

    void setTorqueLimit(double torqueNm);

    void periodic();

    boolean atSetpoint();

    boolean profileDone();

    void setStaticTorque(double torqueNm);

}