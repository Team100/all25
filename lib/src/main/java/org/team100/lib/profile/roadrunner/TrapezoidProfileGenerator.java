package org.team100.lib.profile.roadrunner;

import java.util.List;

import org.team100.lib.util.Math100;

/**
 * Motion profile generator with arbitrary start and end motion states and
 * infinite jerk.
 */
public class TrapezoidProfileGenerator {
    public static MotionProfile generateSimpleMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            boolean overshoot) {
        if (goal.getX() < start.getX()) {
            // ensure the goal is always after the start; plan the flipped profile otherwise
            return generateSimpleMotionProfile(start.flipped(), goal.flipped(), maxVel, maxAccel, overshoot)
                    .flipped();
        }
        return trapezoid(start, goal, maxVel, maxAccel, overshoot);
    }
    
    /////////////////////////////////////////////////

    /** Acceleration-limited profile (trapezoidal) */
    private static MotionProfile trapezoid(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            boolean overshoot) {
        double requiredAccel = (goal.getV() * goal.getV() - start.getV() * start.getV())
                / (2 * (goal.getX() - start.getX()));

        MotionProfile accelProfile = unlimitedJerk(start, maxVel, maxAccel);
        MotionProfile decelProfile = unlimitedJerk(
                new MotionState(
                        goal.getX(),
                        goal.getV(),
                        -goal.getA(),
                        goal.getJ()),
                maxVel,
                maxAccel)
                .reversed();

        MotionProfile noCoastProfile = accelProfile.append(decelProfile);
        double remainingDistance = goal.getX() - noCoastProfile.end().getX();

        if (remainingDistance >= 0.0) {
            return threeSegment(start, maxVel, accelProfile, decelProfile, remainingDistance);

        } else if (Math.abs(requiredAccel) > maxAccel) {
            // not possible within constraints
            if (overshoot) {
                // TODO: is this most efficient? (do we care?)
                return noCoastProfile.append(generateSimpleMotionProfile(
                        noCoastProfile.end(), goal, maxVel, maxAccel, true));
            } else {
                // violate the constraints
                // TODO: remove this
                // single segment profile
                double dt = (goal.getV() - start.getV()) / requiredAccel;
                return new MotionProfileBuilder(start)
                        .appendAccelerationSegment(requiredAccel, dt)
                        .build();
            }
        } else if (start.getV() > maxVel && goal.getV() > maxVel) {
            // this should not happen. TODO: remove this
            // decel, accel
            List<Double> roots = Math100.solveQuadratic(
                    -maxAccel,
                    2 * start.getV(),
                    (goal.getV() * goal.getV() - start.getV() * start.getV()) / (2 * maxAccel) - goal.getX()
                            + start.getX());
            double deltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare).orElseThrow();
            double deltaT3 = Math.abs(start.getV() - goal.getV()) / maxAccel + deltaT1;

            return new MotionProfileBuilder(start)
                    .appendAccelerationSegment(-maxAccel, deltaT1)
                    .appendAccelerationSegment(maxAccel, deltaT3)
                    .build();
        } else {
            // accel, decel
            List<Double> roots = Math100.solveQuadratic(
                    maxAccel,
                    2 * start.getV(),
                    (start.getV() * start.getV() - goal.getV() * goal.getV()) / (2 * maxAccel) - goal.getX()
                            + start.getX());
            double deltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare).orElseThrow();
            double deltaT3 = Math.abs(start.getV() - goal.getV()) / maxAccel + deltaT1;

            return new MotionProfileBuilder(start)
                    .appendAccelerationSegment(maxAccel, deltaT1)
                    .appendAccelerationSegment(-maxAccel, deltaT3)
                    .build();
        }
    }

    /**
     * Normal 3-segment profile: accel, cruise, decel, with unlimited jerk in
     * between.
     */
    private static MotionProfile threeSegment(
            MotionState start,
            double maxVel,
            MotionProfile accelProfile,
            MotionProfile decelProfile,
            double remainingDistance) {
        double deltaT2 = remainingDistance / maxVel;
        return new MotionProfileBuilder(start)
                .appendProfile(accelProfile)
                .appendVelocitySegment(deltaT2)
                .appendProfile(decelProfile)
                .build();
    }

    /**
     * Returns a profile with one max-acceleration segment to full velocity.
     */
    static MotionProfile unlimitedJerk(MotionState start, double maxVel, double maxAccel) {
        double deltaT1 = Math.abs(start.getV() - maxVel) / maxAccel;
        MotionProfileBuilder builder = new MotionProfileBuilder(start);
        if (start.getV() > maxVel) {
            // we need to decelerate
            builder.appendAccelerationSegment(-maxAccel, deltaT1);
        } else {
            builder.appendAccelerationSegment(maxAccel, deltaT1);
        }
        return builder.build();
    }
}
