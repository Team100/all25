package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;

/** Represents motor-shaft encoder, probably some kind of built-in. */
public interface IncrementalBareEncoder extends Glassy {

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * Note some rate implementations can be noisy.
     * 
     * If the encoder can't return a valid measurement (e.g. because hardware is not
     * connected), return empty.
     * 
     * @return rad/s
     */
    OptionalDouble getVelocityRad_S();

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * If the encoder can't return a valid measurement (e.g. because hardware is not
     * connected), return empty.
     * 
     * @return rad
     */
    OptionalDouble getPositionRad();

    /**
     * Resets position to zero.
     *
     * Caches should also be flushed, so the new value is available immediately.
     */
    void reset();

    /**
     * Releases the encoder resource, if necessary (e.g. HAL ports).
     */
    void close();

    /**
     * Sets the incremental encoder position. This is only used to "zero" it, and
     * only done by the ProxyRotaryPositionSensor.
     * 
     * This is very slow, only use it on startup.
     * 
     * Caches should also be flushed, so the new value is available immediately.
     */
    void setEncoderPositionRad(double motorPositionRad);

    /** For logging */
    void periodic();

}
