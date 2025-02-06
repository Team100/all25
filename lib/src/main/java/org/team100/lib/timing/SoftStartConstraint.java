package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Limit acceleration at low speed, to give wheels time to orient.
 * 
 * Say the wheels take about 0.2 sec to orient in the worst case (90 degree
 * error).
 * 
 * Say we want to limit the cross-track error to 0.01 m.
 * 
 * Assume constant acceleration.
 * 
 * x = 1/2 * a * t^2
 * 
 * so the maximum allowed acceleration would be about 0.5 m/s, which would
 * produce
 * a speed of 0.1 m/s during the period.
 * 
 * So the rule is to constrain acceleration to 0.5 m/s if the speed is below 0.1
 * m/s.
 * 
 * TODO: redo this math with the actual swerve kinodynamics parameters.
 */
public class SoftStartConstraint implements TimingConstraint {
    private static final double kMaxCrossTrackM = 0.01;
    private static final double kAlignmentTimeS = 0.2;
    private static final double kAccelLimit = 2.0 * kMaxCrossTrackM / (kAlignmentTimeS * kAlignmentTimeS);
    private static final double kVeloLimit = kAccelLimit * kAlignmentTimeS;

    /** This constraint does not limit velocity. */
    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        return new NonNegativeDouble(Double.POSITIVE_INFINITY);
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocityM_S) {
        if (velocityM_S < kVeloLimit)
            return new MinMaxAcceleration(Double.NEGATIVE_INFINITY, kAccelLimit);
        return MinMaxAcceleration.kNoLimits;

    }

}
