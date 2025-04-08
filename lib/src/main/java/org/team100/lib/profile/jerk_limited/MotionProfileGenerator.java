package org.team100.lib.profile.jerk_limited;

import java.util.List;

import org.team100.lib.util.Math100;

/**
 * Motion profile generator with arbitrary start and end motion states and
 * either dynamic constraints or jerk limiting.
 */
public class MotionProfileGenerator {

    /**
     * Generates a simple motion profile with constant [maxVel], [maxAccel], and
     * [maxJerk]. If [maxJerk] is zero, an
     * acceleration-limited profile will be generated instead of a jerk-limited one.
     * If constraints can't be obeyed,
     * there are two possible fallbacks: If [overshoot] is true, then two profiles
     * will be concatenated (the first one
     * overshoots the goal and the second one reverses back to reach the goal).
     * Otherwise, the highest order constraint
     * (e.g., max jerk for jerk-limited profiles) is repeatedly violated until the
     * goal is achieved.
     *
     * @param start     start motion state
     * @param goal      goal motion state
     * @param maxVel    maximum velocity
     * @param maxAccel  maximum acceleration
     * @param maxJerk   maximum jerk
     * @param overshoot if true overshoot otherwise violate constraints (see
     *                  description above)
     */

    public static MotionProfile generateSimpleMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            boolean overshoot) {
        return generateSimpleMotionProfile(start, goal, maxVel, maxAccel, 0.0, overshoot);
    }

    public static MotionProfile generateSimpleMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel) {
        return generateSimpleMotionProfile(start, goal, maxVel, maxAccel, 0.0);
    }

    public static MotionProfile generateSimpleMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            double maxJerk) {
        return generateSimpleMotionProfile(start, goal, maxVel, maxAccel, maxJerk, false);
    }

    public static MotionProfile generateSimpleMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            double maxJerk,
            boolean overshoot) {
        // ensure the goal is always after the start; plan the flipped profile otherwise
        if (goal.getX() < start.getX()) {
            return generateSimpleMotionProfile(
                    start.flipped(),
                    goal.flipped(),
                    maxVel,
                    maxAccel,
                    maxJerk).flipped();
        }

        if (Math.abs(maxJerk) < 1e-6) {
            // acceleration-limited profile (trapezoidal)
            double requiredAccel = (goal.getV() * goal.getV() - start.getV() * start.getV())
                    / (2 * (goal.getX() - start.getX()));

            MotionProfile accelProfile = generateAccelProfile(start, maxVel, maxAccel);
            MotionProfile decelProfile = generateAccelProfile(
                    new MotionState(
                            goal.getX(),
                            goal.getV(),
                            -goal.getA(),
                            goal.getJ()),
                    maxVel,
                    maxAccel,
                    maxJerk)
                    .reversed();

            MotionProfile noCoastProfile = accelProfile.plus(decelProfile);
            double remainingDistance = goal.getX() - noCoastProfile.end().getX();

            if (remainingDistance >= 0.0) {
                // normal 3-segment profile works
                double deltaT2 = remainingDistance / maxVel;

                return new MotionProfileBuilder(start)
                        .appendProfile(accelProfile)
                        .appendAccelerationControl(0.0, deltaT2)
                        .appendProfile(decelProfile)
                        .build();
            } else if (Math.abs(requiredAccel) > maxAccel) {
                if (overshoot) {
                    // TODO: is this most efficient? (do we care?)
                    return noCoastProfile.plus(generateSimpleMotionProfile(
                            noCoastProfile.end(),
                            goal,
                            maxVel,
                            maxAccel,
                            overshoot = true));
                } else {
                    // single segment profile
                    double dt = (goal.getV() - start.getV()) / requiredAccel;
                    return new MotionProfileBuilder(start)
                            .appendAccelerationControl(requiredAccel, dt)
                            .build();
                }
            } else if (start.getV() > maxVel && goal.getV() > maxVel) {
                // decel, accel
                List<Double> roots = Math100.solveQuadratic(
                        -maxAccel,
                        2 * start.getV(),
                        (goal.getV() * goal.getV() - start.getV() * start.getV()) / (2 * maxAccel) - goal.getX()
                                + start.getX());
                double deltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare).orElseThrow();
                double deltaT3 = Math.abs(start.getV() - goal.getV()) / maxAccel + deltaT1;

                return new MotionProfileBuilder(start)
                        .appendAccelerationControl(-maxAccel, deltaT1)
                        .appendAccelerationControl(maxAccel, deltaT3)
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
                        .appendAccelerationControl(maxAccel, deltaT1)
                        .appendAccelerationControl(-maxAccel, deltaT3)
                        .build();
            }
        } else {
            // jerk-limited profile (S-curve)
            MotionProfile accelerationProfile = generateAccelProfile(start, maxVel, maxAccel, maxJerk);
            // we leverage symmetry here; deceleration profiles are just reversed
            // acceleration ones with the goal
            // acceleration flipped
            MotionProfile decelerationProfile = generateAccelProfile(
                    new MotionState(
                            goal.getX(),
                            goal.getV(),
                            -goal.getA(),
                            goal.getJ()),
                    maxVel,
                    maxAccel,
                    maxJerk)
                    .reversed();

            MotionProfile noCoastProfile = accelerationProfile.plus(decelerationProfile);
            double remainingDistance = goal.getX() - noCoastProfile.end().getX();

            if (remainingDistance >= 0.0) {
                // we just need to add a coast segment of appropriate duration
                double deltaT4 = remainingDistance / maxVel;

                return new MotionProfileBuilder(start)
                        .appendProfile(accelerationProfile)
                        .appendJerkControl(0.0, deltaT4)
                        .appendProfile(decelerationProfile)
                        .build();
            } else {
                // the profile never reaches maxV
                // thus, we need to compute the peak velocity (0 < peak vel < max vel)
                // we *could* construct a large polynomial expression (i.e., a nasty cubic) and
                // solve it using Cardano's
                // method, some kind of inclusion method like modified Anderson-Bjorck-King, or
                // a host of other methods
                // (see https://link.springer.com/content/pdf/bbm%3A978-3-642-05175-3%2F1.pdf
                // for modified ABK)
                // instead, however, we conduct a binary search as it's sufficiently performant
                // for this use case,
                // requires less code, and is overall significantly more comprehensible
                double upperBound = maxVel;
                double lowerBound = 0.0;
                int iterations = 0;
                while (iterations < 1000) {
                    double peakVel = (upperBound + lowerBound) / 2;

                    MotionProfile searchAccelProfile = generateAccelProfile(start, peakVel, maxAccel, maxJerk);
                    MotionProfile searchDecelProfile = generateAccelProfile(goal, peakVel, maxAccel, maxJerk)
                            .reversed();

                    MotionProfile searchProfile = searchAccelProfile.plus(searchDecelProfile);

                    double error = goal.getX() - searchProfile.end().getX();

                    if (Math.abs(error) < 1e-6) {
                        return searchProfile;
                    }

                    if (error > 0.0) {
                        // we undershot so shift the lower bound up
                        lowerBound = peakVel;
                    } else {
                        // we overshot so shift the upper bound down
                        upperBound = peakVel;
                    }

                    iterations++;
                }

                // constraints are not satisfiable
                if (overshoot) {
                    return noCoastProfile.plus(generateSimpleMotionProfile(
                            noCoastProfile.end(),
                            goal,
                            maxVel,
                            maxAccel,
                            maxJerk,
                            overshoot = true));
                } else {
                    // violate max jerk first
                    return generateSimpleMotionProfile(
                            start,
                            goal,
                            maxVel,
                            maxAccel,
                            overshoot = false);
                }
            }
        }
    }

    private static MotionProfile generateAccelProfile(
            MotionState start,
            double maxVel,
            double maxAccel) {
        return generateAccelProfile(
                start, maxVel, maxAccel, 0.0);
    }

    private static MotionProfile generateAccelProfile(
            MotionState start,
            double maxVel,
            double maxAccel,
            double maxJerk) {
        if (Math.abs(maxJerk) < 1e-6) {
            // acceleration-limited
            double deltaT1 = Math.abs(start.getV() - maxVel) / maxAccel;
            MotionProfileBuilder builder = new MotionProfileBuilder(start);
            if (start.getV() > maxVel) {
                // we need to decelerate
                builder.appendAccelerationControl(-maxAccel, deltaT1);
            } else {
                builder.appendAccelerationControl(maxAccel, deltaT1);
            }
            return builder.build();
        } else {
            // jerk-limited
            // compute the duration and velocity of the first segment
            double deltaT1;
            double deltaV1;
            if (start.getA() > maxAccel) {
                // slow down and see where we are
                deltaT1 = (start.getA() - maxAccel) / maxJerk;
                deltaV1 = start.getA() * deltaT1 - 0.5 * maxJerk * deltaT1 * deltaT1;
            } else {
                // otherwise accelerate
                deltaT1 = (maxAccel - start.getA()) / maxJerk;
                deltaV1 = start.getA() * deltaT1 + 0.5 * maxJerk * deltaT1 * deltaT1;
            }

            // compute the duration and velocity of the third segment
            double deltaT3 = maxAccel / maxJerk;
            double deltaV3 = maxAccel * deltaT3 - 0.5 * maxJerk * deltaT3 * deltaT3;

            // compute the velocity change required in the second segment
            double deltaV2 = maxVel - start.getV() - deltaV1 - deltaV3;

            if (deltaV2 < 0.0) {
                // there is no constant acceleration phase
                // the second case checks if we're going to exceed max vel
                if (start.getA() > maxAccel
                        || (start.getV() - maxVel) > (start.getA() * start.getA()) / (2 * maxJerk)) {
                    // problem: we need to cut down on our acceleration but we can't cut our initial
                    // decel
                    // solution: we'll lengthen our initial decel to -max accel and similarly with
                    // our final accel
                    // if this results in an over correction, decel instead to a good accel
                    double newDeltaT1 = (start.getA() + maxAccel) / maxJerk;
                    double newDeltaV1 = start.getA() * newDeltaT1 - 0.5 * maxJerk * newDeltaT1 * newDeltaT1;

                    double newDeltaV2 = maxVel - start.getV() - newDeltaV1 + deltaV3;

                    if (newDeltaV2 > 0.0) {
                        // we decelerated too much
                        List<Double> roots = Math100.solveQuadratic(
                                -maxJerk,
                                2 * start.getA(),
                                start.getV() - maxVel - start.getA() * start.getA() / (2 * maxJerk));
                        double finalDeltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare)
                                .orElseThrow();
                        double finalDeltaT3 = finalDeltaT1 - start.getA() / maxJerk;

                        return new MotionProfileBuilder(start)
                                .appendJerkControl(-maxJerk, finalDeltaT1)
                                .appendJerkControl(maxJerk, finalDeltaT3)
                                .build();
                    } else {
                        // we're almost good
                        double newDeltaT2 = newDeltaV2 / -maxAccel;

                        return new MotionProfileBuilder(start)
                                .appendJerkControl(-maxJerk, newDeltaT1)
                                .appendJerkControl(0.0, newDeltaT2)
                                .appendJerkControl(maxJerk, deltaT3)
                                .build();
                    }
                } else {
                    // cut out the constant accel phase and find a shorter delta t1 and delta t3
                    List<Double> roots = Math100.solveQuadratic(
                            maxJerk,
                            2 * start.getA(),
                            start.getV() - maxVel + start.getA() * start.getA() / (2 * maxJerk));
                    double newDeltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare).orElseThrow();
                    double newDeltaT3 = newDeltaT1 + start.getA() / maxJerk;

                    return new MotionProfileBuilder(start)
                            .appendJerkControl(maxJerk, newDeltaT1)
                            .appendJerkControl(-maxJerk, newDeltaT3)
                            .build();
                }
            } else {
                // there is a constant acceleration phase
                double deltaT2 = deltaV2 / maxAccel;

                MotionProfileBuilder builder = new MotionProfileBuilder(start);
                if (start.getA() > maxAccel) {
                    builder.appendJerkControl(-maxJerk, deltaT1);
                } else {
                    builder.appendJerkControl(maxJerk, deltaT1);
                }
                return builder.appendJerkControl(0.0, deltaT2)
                        .appendJerkControl(-maxJerk, deltaT3)
                        .build();
            }
        }
    }
}