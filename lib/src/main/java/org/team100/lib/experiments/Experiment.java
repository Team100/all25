package org.team100.lib.experiments;

/**
 * An experiment is something that can be selectively enabled.
 */
public enum Experiment {
    /**
     * Smooth chassis speeds.
     */
    UseSetpointGenerator,
    /**
     * Flush network tables as often as possible. Do not enable this experiment in
     * competition, you'll overwhelm the network and the RIO
     */
    FlushOften,
    /**
     * Pay attention to camera input. It's useful to turn this off for testing and
     * calibration.
     */
    HeedVision,
    /**
     * Control heading when manually steering in snaps mode, to prevent
     * rotational drifting.
     */
    StickyHeading,
    /**
     * Drive to note turns toward the note
     */
    DriveToNoteWithRotation,
    /**
     * Use softer vision update gains
     */
    AvoidVisionJitter,
    /**
     * Filter snap rotational output to remove oscillation
     */
    SnapThetaFilter,
    /**
     * Make the kinodynamic limiter prefer rotation.
     */
    LimitsPreferRotation,
    /**
     * Clip the snap omega
     */
    SnapGentle,
    /**
     * Ignore very small inputs, to reduce jittering.
     */
    SwerveDeadband,
    /**
     * Help drive motors overcome steering.
     * TODO(2/24/25) I think this doesn't help, should be deleted.
     */
    CorrectSpeedForSteering,
    /**
     * Use the steering error to adjust the drive motor speed, minimizing
     * cross-track error. This could be the "cosine" used by some teams. It serves
     * the same function as the "steer at rest" idea -- avoid going in the wrong
     * direction. In this case, since the upper layers are unaware of it, it will
     * introduce lag (thus controller error) into timed movements. But it has the
     * benefit of being simple.
     */
    ReduceCrossTrackError,
    /**
     * The ReferenceController will try to align the steering perfectly before
     * starting motion.
     * TODO(2/24/25) I think this isn't worth the coupling between layers
     */
    SteerAtRest,
    /**
     * Use pure outboard PID for steering control, rather than the usual profiled
     * motion -- it's faster and less work for the RoboRIO.
     */
    UnprofiledSteering
}
