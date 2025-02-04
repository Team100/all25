package org.team100.lib.sensors;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Three-axis gyro, NWU.
 */
public interface Gyro extends Glassy {
    /**
     * Yaw in radians, NWU, counterclockwise positive.
     * Implementations should extrapolate using the yaw rate,
     * to get the yaw at the current Takt time.
     */
    Rotation2d getYawNWU();

    /** Yaw rate in rad/s, NWU, counterclockwise positive. */
    double getYawRateNWU();

    /** Pitch in radians, NWU, positive-down. */
    Rotation2d getPitchNWU();

    /** Roll in radians, NWU, positive-right. */
    Rotation2d getRollNWU();

    /** For computing rate. */
    void periodic();

}
